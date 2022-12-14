/*
    This file is part of lgob.

    lgob is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    lgob is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with lgob.  If not, see <http://www.gnu.org/licenses/>.

    Copyright (C) 2008 - 2014 Lucas Hermann Negri
*/

static void priv_value_push(lua_State* L, GValue* value, gboolean copy);

/* Signal handling */

static void* priv_callback_handle(Data* data, ...)
{
    lua_State* L = data->L;
    va_list args;
    int c, initial_size = lua_gettop(L);
    gchar* error = NULL;
    
    if(lua_status(L) == LUA_YIELD) {
        g_warning("Attempt to handle a signal in a suspended coroutine");
        return NULL;
    }

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    #ifdef IDEBUG
    fprintf(stderr, "Handling a callback with %i args\n", data->signal_info.n_params);
    #endif

    va_start(args, data);

    /* Push the other args */
    for(c = 1; c <= data->signal_info.n_params; c++)
    {
        /* GObject api says that it's "mangled", so: */
        GType type = data->signal_info.param_types[c - 1] & ~G_SIGNAL_TYPE_STATIC_SCOPE;

        #ifdef IDEBUG
        fprintf(stderr, "Arg[%i] type = %s\n", c - 1, g_type_name(type));
        #endif

        GValue value = {0};
        g_value_init(&value, type);
        G_VALUE_COLLECT(&value, args, G_VALUE_NOCOPY_CONTENTS, &error);

        if(G_UNLIKELY(error))
            luaL_error(L, "priv_callback_handle error! %s", error);

        priv_value_push(L, &value, FALSE); /* Local scope only! */
        g_value_unset(&value);
    }

    va_end(args);
    
    /* Call the function */
    void* ret = 0;
    GType ret_type = data->signal_info.return_type & ~G_SIGNAL_TYPE_STATIC_SCOPE;
    lua_call(L, data->signal_info.n_params + 1, ret_type == G_TYPE_NONE ? 0 : 1);
    
    /* Get the return */
    switch(ret_type)
    {
        case G_TYPE_BOOLEAN : ret = GINT_TO_POINTER(lua_toboolean(L, -1));  break;
        case G_TYPE_INT     : ret = GINT_TO_POINTER(lua_tointeger(L, -1));  break;
        case G_TYPE_STRING  : ret = (void*)lua_tostring(L, -1);             break;
        case G_TYPE_OBJECT  : 
        {
            Object* obj = lua_touserdata(L, -1);
            ret = obj->pointer;
            break;
        }
    }

    #ifdef IDEBUG
    fprintf(stderr, "Returning from type %s\n", g_type_name(ret_type));
    #endif

    /* Leave the stack with the initial content (balanced operation)*/
    lua_settop(L, initial_size);

    return ret;
}

/* To avoid some compiler warnings and maybe some wrong optimizations. */
void fake()
{
    priv_callback_handle(NULL);
}

static void priv_callback_free(gpointer user_data, GClosure* closure)
{
    #ifdef IDEBUG
    fprintf(stderr, "Releasing Callback data!\n");
    #endif

    Data* data = (Data*)user_data;

    /* Remove refs from Lua side */
    luaL_unref(data->L, LUA_REGISTRYINDEX, data->function_ref);
    luaL_unref(data->L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Free the data */
    g_free(data);
}

static guint priv_idle_add(gint priority, Data* userdata)
{
    guint res = g_idle_add_full(priority, (GSourceFunc)priv_callback_handle,
        userdata, (GDestroyNotify)priv_callback_free);

    return res;
}

static guint priv_timeout_add(gint priority, guint interval, Data* userdata)
{
    guint res = g_timeout_add_full(priority, interval,
        (GSourceFunc)priv_callback_handle, userdata, (GDestroyNotify)priv_callback_free);

    return res;
}

static GClosure* priv_closure_new(Data* data)
{
    #ifdef IDEBUG
    fprintf(stderr, "Creating a closure\n");
    #endif

    return g_cclosure_new_swap(G_CALLBACK(priv_callback_handle), data, priv_callback_free);
}

static void priv_object_connect(lua_State* L, Object* obj, const gchar* signal,
    gint function_ref, gint ud_ref, gboolean after)
{
    /* Create the callback data */
    Data* data = g_malloc(sizeof(Data));
    data->L = L;
    data->function_ref = function_ref;
    data->ud_ref = ud_ref;

    guint signal_id;
    gulong handler_id;
    GQuark detail = 0;
    GType type = G_TYPE_FROM_INSTANCE(obj->pointer);
    const gchar* name =  g_type_name(type);

    if(!g_signal_parse_name(signal, type, &signal_id, &detail, TRUE))
        /* Wrong signal / object */
        luaL_error(L, "The GObject %s doesn't have the signal %s!", name, signal);

    /* Get some info about the signal */
    GSignalQuery signal_info;
    g_signal_query(signal_id, &signal_info);
    data->signal_info.param_types = signal_info.param_types;
    data->signal_info.return_type = signal_info.return_type;
    data->signal_info.n_params = signal_info.n_params;

    /* Connect the signal and push the id to Lua side */
    GClosure* closure = priv_closure_new(data);
    handler_id = g_signal_connect_closure_by_id(obj->pointer, signal_id, detail, closure, after);
    lua_pushnumber(L, handler_id);

    #ifdef IDEBUG
    fprintf(stderr, "Connected the signal %s (%i params) of a %s\n", signal,
        data->signal_info.n_params, name);
    #endif
}

/* GValue handling */

static void priv_value_push(lua_State* L, GValue* value, gboolean copy)
{
    GType type = value->g_type;

    #ifdef IDEBUG
    fprintf(stderr, "\tPushing a %s!\n", g_type_name(type));
    #endif

    /* A fundamental type? */
    if(G_LIKELY(G_TYPE_IS_FUNDAMENTAL(type)))
    {
        switch(type)
        {
            case G_TYPE_NONE:
            case G_TYPE_INVALID:
            {
                lua_pushnil(L);
                return;
            }
            case G_TYPE_CHAR:
            {
                gchar lval = value->data[0].v_int;
                lua_pushlstring(L, &lval, 1);
                return;
            }
            case G_TYPE_UCHAR:
            {
                gchar lval = value->data[0].v_uint;
                lua_pushlstring(L, &lval, 1);
                return;
            }
            case G_TYPE_BOOLEAN:
            {
                lua_pushboolean(L, value->data[0].v_int);
                return;
            }
            case G_TYPE_INT:
            {
                lua_pushnumber(L, value->data[0].v_int);
                return;
            }
            case G_TYPE_UINT:
            {
                lua_pushnumber(L, value->data[0].v_uint);
                return;
            }
            case G_TYPE_LONG:
            {
                lua_pushnumber(L, value->data[0].v_long);
                return;
            }
            case G_TYPE_ULONG:
            {
                lua_pushnumber(L, value->data[0].v_ulong);
                return;
            }
            case G_TYPE_INT64:
            {
                lua_pushnumber(L, value->data[0].v_int64);
                return;
            }
            case G_TYPE_UINT64:
            {
                lua_pushnumber(L, value->data[0].v_uint64);
                return;
            }
            case G_TYPE_FLOAT:
            {
                lua_pushnumber(L, value->data[0].v_float);
                return;
            }
            case G_TYPE_DOUBLE:
            {
                lua_pushnumber(L, value->data[0].v_double);
                return;
            }
            case G_TYPE_STRING:
            {
                lua_pushstring(L, (gchar*)value->data[0].v_pointer);
                return;
            }
        }
    }

    /* OK, needs special care! */

    if(g_type_is_a(type, G_TYPE_BOXED))
    {
        #ifdef IDEBUG
        fprintf(stderr, "\tBoxed type!\n");
        #endif

        /* char** (always copy) */
        if(type == G_TYPE_STRV)
        {
            GStrv lval = (GStrv)value->data[0].v_pointer;

            if(lval)
            {
                lua_pushliteral(L, "lgobGstrv2table");
                lua_rawget(L, LUA_REGISTRYINDEX);
                lua_pushlightuserdata(L, lval);
                lua_call(L, 1, 1);
            }
            else
                lua_pushnil(L);

            return;
        }

        if(copy)
            priv_boxed_new(L, value->g_type, value->data[0].v_pointer);
        else
            priv_boxed_new_no_copy(L, value->g_type, value->data[0].v_pointer);

        return;
    }

    /* Enum or flag? */
    if(g_type_is_a(type, G_TYPE_ENUM) || g_type_is_a(type, G_TYPE_FLAGS))
    {
        lua_pushnumber(L, value->data[0].v_int);
        return;
    }

    /* Object? */
    if(g_type_is_a(type, G_TYPE_OBJECT))
    {
        if(copy)
            priv_object_new(L, value->data[0].v_pointer, FALSE);
        else
            priv_object_new_no_ref(L, value->data[0].v_pointer);

        return;
    }

    /* Struct? I would love to put it all in a switch... */
    if(type != G_TYPE_NONE)
    {
        /* "Structs" are just a pointer to the real data struct */
        gpointer lval = value->data[0].v_pointer;
        priv_generic_struct_new(L, lval, FALSE);
        return;
    }

    g_warning("lgob_value_push error: Type not handled!");
    lua_pushnil(L);
}

static void priv_value_set(lua_State* L, GValue* value, gint index)
{
    GType type = value->g_type;

    #ifdef IDEBUG
    fprintf(stderr, "Setting a %s with value at index %i!\n", g_type_name(type), index);
    #endif

    /* A fundamental type? */
    if(G_LIKELY(G_TYPE_IS_FUNDAMENTAL(type)))
    {
        switch(type)
        {
            case G_TYPE_CHAR:
            {
                value->data[0].v_int = lua_tostring(L, index)[0];
                return;
            }
            case G_TYPE_UCHAR:
            {
                value->data[0].v_uint = g_utf8_get_char(lua_tostring(L, index));
                return;
            }
            case G_TYPE_BOOLEAN:
            {
                value->data[0].v_int = lua_toboolean(L, index);
                return;
            }
            case G_TYPE_INT:
            {
                value->data[0].v_int = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_UINT:
            {
                value->data[0].v_uint = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_LONG:
            {
                value->data[0].v_long = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_ULONG:
            {
                value->data[0].v_ulong = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_INT64:
            {
                value->data[0].v_int64 = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_UINT64:
            {
                value->data[0].v_uint64 = lua_tointeger(L, index);
                return;
            }
            case G_TYPE_FLOAT:
            {
                value->data[0].v_float = lua_tonumber(L, index);
                return;
            }
            case G_TYPE_DOUBLE:
            {
                value->data[0].v_double = lua_tonumber(L, index);
                return;
            }
            case G_TYPE_STRING:
            {
                g_value_set_string(value, lua_tostring(L, index));
                return;
            }
        }
    }

    /* OK, needs special care! */

    /* Boxed types */
    if(g_type_is_a(type, G_TYPE_BOXED))
    {
        #ifdef IDEBUG
        fprintf(stderr, "\tBoxed type!\n");
        #endif

        /* char** */
        if(type == G_TYPE_STRV)
        {
            lua_getfield(L, LUA_REGISTRYINDEX, "lgobTable2gstrv");

            if(!lua_isnil(L, index))
            {
                lua_pushvalue(L, index);
                lua_call(L, 1, 1);
                GStrv strv = lua_touserdata(L, -1);
                lua_pop(L, 1);
                value->data[0].v_pointer = strv;
            }
            else
                value->data[0].v_pointer = NULL;

            return;
        }

        /* Generic boxed type */
        Object* obj = lua_touserdata(L, index);
        g_value_set_boxed(value, obj->pointer);

        return;
    }

    /* Enum? */
    if(g_type_is_a(type, G_TYPE_ENUM) || g_type_is_a(type, G_TYPE_FLAGS))
    {
        value->data[0].v_int = lua_tointeger(L, index);
        return;
    }

    /* Object? */
    if(g_type_is_a(type, G_TYPE_OBJECT))
    {
        Object* obj = lua_touserdata(L, index);

        if(obj)
        {
            value->data[0].v_pointer = obj->pointer;
            g_object_ref(obj->pointer);
        }
        else
            value->data[0].v_pointer = NULL;

        return;
    }

    /* Struct? I would love to put it all in a switch... */
    if(type != G_TYPE_NONE)
    {
        Object* obj = lua_touserdata(L, index);

        if(obj)
            value->data[0].v_pointer = obj->pointer;
        else
            value->data[0].v_pointer = NULL;

        return;
    }

    g_warning("lgob_value_set error: Type not handled!");
}

static int lgob_value_push(lua_State* L)
{
    GValue* value = lua_touserdata(L, -1);
    priv_value_push(L, value, TRUE);

    return 1;
}

static int lgob_value_set(lua_State* L)
{
    GValue* value = lua_touserdata(L, 1);
    priv_value_set(L, value, 2);

    return 0;
}

static int lgob_object_new(lua_State* L)
{
    priv_object_new(L, lua_touserdata(L, 1), lua_toboolean(L, 2));
    return 1;
}

static int lgob_struct_new(lua_State* L)
{
    priv_generic_struct_new(L, lua_touserdata(L, 1), lua_toboolean(L, 2));
    return 1;
}

static int lgob_object_set(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    
    int i, size = lua_gettop(L);
    Object* obj = lua_touserdata(L, 1);
    const char* property = NULL;
    gpointer klass = G_OBJECT_GET_CLASS(obj->pointer);

    /* Set the properties, pair by pair */
    for(i = 2; i <= size; i += 2)
    {
        /* Get the property */
        property = luaL_checkstring(L, i);

        /* Set the value */
        GParamSpec* spec = g_object_class_find_property(klass, property);

        #ifdef IDEBUG
        fprintf(stderr, "Setting the %s property, of type %s!\n", property, g_type_name(spec->value_type));
        #endif

        if(G_UNLIKELY(!spec))
            luaL_error(L, "The GObject doesn't have the property %s!", property);

        GValue val = {0};
        g_value_init(&val, spec->value_type);
        priv_value_set(L, &val, i + 1);
        g_object_set_property(obj->pointer, property, &val);
        g_value_unset(&val);
    }

    return 0;
}

static int lgob_object_get(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    
    int i, size = lua_gettop(L);
    Object* obj = lua_touserdata(L, 1);
    const char* property = NULL;
    gpointer klass = G_OBJECT_GET_CLASS(obj->pointer);

    for(i = 2; i <= size; ++i)
    {
        /* Get the property */
        property = luaL_checkstring(L, i);

        GParamSpec* spec = g_object_class_find_property(klass, property);

        #ifdef IDEBUG
        fprintf(stderr, "Getting the %s property, of type %s!\n", property, g_type_name(spec->value_type));
        #endif

        if(G_UNLIKELY(!spec))
            luaL_error(L, "The GObject doesn't have the property %s!", property);

        /* Get the value */
        GValue val = {0};
        g_value_init(&val, spec->value_type);
        g_object_get_property(obj->pointer, property, &val);
        priv_value_push(L, &val, TRUE);
        g_value_unset(&val);
    }

    return i - 2;
}

static int lgob_object_connect(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TSTRING);
    luaL_checktype(L, 3, LUA_TFUNCTION);

    int size = lua_gettop(L);
    int ud_ref = -1;
    gboolean after = FALSE;

    if(size > 3)
    {
        /* If have a fourth parameter, then it is the user data */
        luaL_checkany(L, 4);

        /* Create a reference to the user data */
        lua_pushvalue(L, 4);
        ud_ref = luaL_ref(L, LUA_REGISTRYINDEX);

        /* Specified after / before ? */
        if(size > 4)
        {
            luaL_checktype(L, 5, LUA_TBOOLEAN);
            after = lua_toboolean(L, 5);
        }
    }

    Object* ptr = lua_touserdata(L, 1);
    const gchar* signal = lua_tostring(L, 2);

    /* Create a reference to the function, for a latter call */
    lua_pushvalue(L, 3);
    int ref = luaL_ref(L, LUA_REGISTRYINDEX);

    priv_object_connect(L, ptr, signal, ref, ud_ref, after);

    return 1;
}

static int lgob_object_disconnect(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TNUMBER);

    Object* obj = lua_touserdata(L, 1);
    gulong id = lua_tointeger(L, 2);

    g_signal_handler_disconnect(obj->pointer, id);
    return 0;
}

static int lgob_object_block(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TNUMBER);

    Object* obj = lua_touserdata(L, 1);
    g_signal_handler_block(obj->pointer, lua_tointeger(L, 2));

    return 0;
}

static int lgob_object_unblock(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TNUMBER);

    Object* obj = lua_touserdata(L, 1);
    g_signal_handler_unblock(obj->pointer, lua_tointeger(L, 2));

    return 0;
}

static int lgob_object_ref(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    Object* obj = lua_touserdata(L, 1);

    g_object_ref(obj->pointer);
    return 0;
}

static int lgob_object_unref(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    Object* obj = lua_touserdata(L, 1);

    g_object_unref(obj->pointer);
    return 0;
}

static int lgob_object_to_string(lua_State* L)
{
    Object* obj = lua_touserdata(L, 1);

    gchar* type;

    if(G_IS_OBJECT(obj->pointer))
        type = (gchar*)g_type_name(G_TYPE_FROM_INSTANCE(obj->pointer));
    else
        type = (gchar*)"Unknown";

    char name[51];
    snprintf(name, 50, "%s: %p", type, obj->pointer);
    lua_pushstring(L, name);

    return 1;
}

static int lgob_object_eq(lua_State* L)
{
    Object* obj1 = lua_touserdata(L, 1);
    Object* obj2 = lua_touserdata(L, 2);

    lua_pushboolean(L, obj1->pointer == obj2->pointer);

    return 1;
}

static int lgob_object_gc(lua_State* L)
{
    Object* obj = lua_touserdata(L, 1);

    #ifdef IDEBUG
    g_assert(G_IS_OBJECT(obj->pointer));
    const gchar* type = g_type_name(G_TYPE_FROM_INSTANCE(obj->pointer));
    fprintf(stderr, "Garbage collecting %s!\n", type);
    #endif

    /* Release our ref */
    g_object_unref(obj->pointer);

    /* If need an extra unref */
    if(obj->need_unref)
        g_object_unref(obj->pointer);

    /*
     * Note that if another Object still holds a ref on the C side to
     * the object, it will not be release here, and will only be released
     * by the C library when its ref count reaches 0.
     */

    #ifdef IDEBUG
    if(G_IS_OBJECT(obj->pointer))
        fprintf(stderr, "%s still alive!\n", type);
    #endif

    return 0;
}

static int lgob_closure_new(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TLIGHTUSERDATA);
    lua_pushlightuserdata(L, priv_closure_new(lua_touserdata(L, 1)));

    return 1;
}

static void priv_set_mt_fields(lua_State* L, gboolean gc_too)
{
    /* the metatable must be on the top */
    
    /* __tostring */
    lua_pushliteral(L, "__tostring");
    lua_getfield(L, LUA_REGISTRYINDEX, "lgobTostring");
    lua_rawset(L, -3);

    /* __eq */
    lua_pushliteral(L, "__eq");
    lua_getfield(L, LUA_REGISTRYINDEX, "lgobEq");
    lua_rawset(L, -3);
    
    if(G_LIKELY(gc_too))
    {
        /* __gc */
        lua_pushliteral(L, "__gc");
        lua_getfield(L, LUA_REGISTRYINDEX, "lgobGc");
        lua_rawset(L, -3);
    }
}

static void priv_get_class(lua_State* L, const gchar* typename)
{
    #ifdef IDEBUG
    fprintf(stderr, "Getting class for typename %s!\n", typename);
    #endif
    
    int top = lua_gettop(L);
    
    /* Check if there's an explicit mapping from the typename to a table name */
    lua_getfield(L, LUA_REGISTRYINDEX, "lgobSpecial");
    lua_getfield(L, -1, typename);
    
    if(G_LIKELY(lua_isnil(L, -1)))
    {
        /* Split GtkWindow into Gtk (prefix) and Window (name) */
        int i; for(i = 1; typename[i] && !g_ascii_isupper(typename[i]); ++i);
    
        /* Get the main table from the prefix table */
        lua_getfield(L, LUA_REGISTRYINDEX, "lgobPrefix");
        lua_pushlstring(L, typename, i); /* prefix */
        lua_gettable(L, -2);
        
        /* Nobody owns that prefix! */
        if(G_UNLIKELY(lua_isnil(L, -1)))
            lua_getfield(L, LUA_REGISTRYINDEX, "lgobObject");
        else
        {
            const char* key = lua_tostring(L, -1);
            
            #ifdef IDEBUG
            fprintf(stderr, "Getting key %s!\n", key);
            #endif
            
            lua_getglobal(L, key);
            lua_pushstring(L, &typename[i]); /* name */
            lua_gettable(L, -2);
            
            /* There's no table with that name! */
            if(G_UNLIKELY(lua_isnil(L, -1)))
            {
                #ifdef IDEBUG
                fprintf(stderr, "No class found, returning lgobObject\n");
                #endif
                lua_getfield(L, LUA_REGISTRYINDEX, "lgobObject");
            }
        }
    }
    
    /* Clean this mess */
    lua_replace(L, top + 1);
    lua_settop(L, top + 1);
}

static gboolean priv_object_new(lua_State* L, GObject* pointer, gboolean constructor)
{
    #ifdef IDEBUG
    fprintf(stderr, "Constructing a new GObject!\n");
    #endif

    // NOTE GObject memory managment is full of quirks
    
    if(G_LIKELY(G_IS_OBJECT(pointer)))
    {
        const gchar* typename = g_type_name(G_TYPE_FROM_INSTANCE(pointer));
        
        /* Avoid floating references */
#if GLIB_MINOR_VERSION >= 10
        gboolean need_unref;
        
        if(G_LIKELY(g_object_is_floating(pointer)) )
        {
            #ifdef IDEBUG
            fprintf(stderr, "Object of type %s is floating!\n", typename);
            #endif
            need_unref = FALSE;
            g_object_ref_sink(pointer);
        }
        else
        {
            #ifdef IDEBUG
            fprintf(stderr, "Object of type %s is not floating!\n", typename);
            #endif
            need_unref = constructor;
            g_object_ref(pointer);
        }
#else
        /* Before gobject .10, floating objects were handled in GTK */
        gboolean need_unref = constructor;
#endif
        
        #ifdef IDEBUG
        fprintf(stderr, "Type %s!\n", typename);
        #endif
        
        Object* obj = lua_newuserdata(L, sizeof(Object));
        obj->pointer = pointer;
        obj->need_unref = need_unref;

        /* Create a shared metatable for the type */
        if(G_UNLIKELY(luaL_newmetatable(L, typename)))
        {
            /* __index */
            lua_pushliteral(L, "__index");
            priv_get_class(L, typename);
            lua_rawset(L, -3);
            
            priv_set_mt_fields(L, TRUE);
        }

        /* Set the metatable */
        lua_setmetatable(L, -2);
        return TRUE;
    }

    #ifdef IDEBUG
    fprintf(stderr, "\tInvalid pointer!\n");
    #endif

    lua_pushnil(L);
    return FALSE;
}

static gboolean priv_object_new_no_ref(lua_State* L, GObject* pointer)
{
    #ifdef IDEBUG
    fprintf(stderr, "Constructing a new GObject (not managed)!\n");
    #endif

    if(G_LIKELY(G_IS_OBJECT(pointer)))
    {
        const gchar* typename = g_type_name(G_TYPE_FROM_INSTANCE(pointer));
        
        #ifdef IDEBUG
        fprintf(stderr, "\tType %s\n", typename);
        #endif

        /* Get the class table */

        Object* obj = lua_newuserdata(L, sizeof(Object));
        obj->pointer = pointer;
        obj->need_unref = FALSE;

        /* Set the metatable */
        if(luaL_newmetatable(L, "lgobObjectNoRef"))
        {
            /* __index */
            lua_pushliteral(L, "__index");
            priv_get_class(L, typename);
            lua_rawset(L, -3);

            priv_set_mt_fields(L, FALSE);
        }
        
        lua_setmetatable(L, -2);
        return TRUE;
    }

    #ifdef IDEBUG
    fprintf(stderr, "\tInvalid pointer!");
    #endif

    lua_pushnil(L);
    return FALSE;
}

static gboolean priv_generic_struct_new(lua_State* L, gpointer pointer, gboolean need_unref)
{
    #ifdef IDEBUG
    fprintf(stderr, "Creating a Struct!\n");
    #endif

    if(pointer) /* Protect agains NULLs */
    {
        Object* st = lua_newuserdata(L, sizeof(Object));
        st->pointer = pointer;
        st->need_unref = need_unref;

        /* Set the metatable */
        if(luaL_newmetatable(L, "lgobstruct"))
        {
            /* A new metatable */
            #ifdef IDEBUG
            fprintf(stderr, "Creating a new metatable!\n");
            #endif

            /* __gc */
            lua_pushliteral(L, "__gc");
            lua_pushcfunction(L, lgob_struct_gc);
            lua_rawset(L, -3);

            /* __tostring */
            lua_pushliteral(L, "__tostring");
            lua_pushcfunction(L, lgob_struct_to_string);
            lua_rawset(L, -3);

            /* __eq */
            lua_pushliteral(L, "__eq");
            lua_pushcfunction(L, lgob_object_eq);
            lua_rawset(L, -3);
        }

        /* Just for debug, the set metatable block is generic */
        #ifdef IDEBUG
        else
        {
            fprintf(stderr, "Using an already created metatable!\n");
        }
        #endif
        
        lua_setmetatable(L, -2);
        return TRUE;
    }

    #ifdef IDEBUG
    fprintf(stderr, "\tInvalid pointer!\n");
    #endif

    lua_pushnil(L);

    return FALSE;
}

static int lgob_struct_gc(lua_State* L)
{
    Object* obj = lua_touserdata(L, 1);

    #ifdef IDEBUG
    fprintf(stderr, "Garbage collecting Struct!\n");
    #endif

    if(obj->need_unref)
    {
        #ifdef IDEBUG
        fprintf(stderr, "Freeing manually allocated pointer!\n");
        #endif
        g_free(obj->pointer);
    }

    return 0;
}

static int lgob_struct_to_string(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    Object* obj = lua_touserdata(L, 1);

    char name[31];
    snprintf(name, 30, "Struct: %p", obj->pointer);
    lua_pushstring(L, name);

    return 1;
}

static int lgob_object_notify(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TSTRING);
    Object* obj = lua_touserdata(L, 1);
    g_object_notify(obj->pointer, lua_tostring(L, 2));
    
    return 0;
}

static int lgob_object_thaw_notify(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    Object* obj = lua_touserdata(L, 1);
    g_object_thaw_notify(obj->pointer);

    return 0;
}

static int lgob_object_freeze_notify(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    Object* obj = lua_touserdata(L, 1);
    g_object_freeze_notify(obj->pointer);

    return 0;
}

static int lgob_object_cast(lua_State* L)
{
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TTABLE);
    
    /* all objects of the same class shared the same metatable, so
       we must give a unique metatable to allow this 'cast', unless
       this object already have been casted before (inheritance), so
       we just reset the index */
    
    lua_getmetatable(L, 1);
    lua_getfield(L, -1, "_unique");
    
    if(lua_toboolean(L, -1) == FALSE)
    {
        lua_newtable(L);
    
        /* set the metatable fields */
        priv_set_mt_fields(L, TRUE);
    
        lua_pushliteral(L, "_unique");
        lua_pushboolean(L, TRUE);
        lua_rawset(L, -3);
    }
    else
        lua_pop(L, 1);
    
    lua_pushliteral(L, "__index");
    lua_pushvalue(L, 2);
    lua_rawset(L, -3);
    
    /* we expect that the table at index 2 to be derived from gobject.Object */
    lua_setmetatable(L, 1);
    
    lua_pushvalue(L, 1);
    return 1;
}
