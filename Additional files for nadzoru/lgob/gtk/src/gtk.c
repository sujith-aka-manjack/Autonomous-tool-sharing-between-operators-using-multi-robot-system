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

/* Lua */
#include <lua.h>
#include <lauxlib.h>

/* GTK+ */
#include <gtk/gtk.h>

/* C */
#include <string.h>

#include <lgob/common/types.h>
#include <lgob/gobject/types.h>

#define _LIB_FREE g_free

/* warning: this is not completely equal to the Lua 5.1 luaL_register! */
#if LUA_VERSION_NUM > 501
static void luaL_register(lua_State* L, const char* libname, const luaL_Reg* reg)
{
    if(libname == NULL)
    {
        luaL_setfuncs(L, reg, 0);
    }
    else
    {
        luaL_newlib(L, reg);
        lua_pushvalue(L, -1);
        lua_setglobal(L, libname);
    }
}
#endif

static void register_class(lua_State* L, const char* name, const char* base, const luaL_Reg* reg)
{
    lua_pushstring(L, name);
    lua_newtable(L);
    luaL_register(L, NULL, reg);

    if(base)
    {
        lua_newtable(L);
        lua_pushliteral(L, "__index");
        lua_pushstring(L, base);
        lua_rawget(L, -6);
        lua_rawset(L, -3);
        lua_setmetatable(L, -2);
    }
    
    lua_rawset(L, -3); 
}

static void object_new(lua_State* L, gpointer ptr, gboolean constructor)
{
    lua_pushliteral(L, "lgobObjectNew");
    lua_rawget(L, LUA_REGISTRYINDEX);
    lua_pushlightuserdata(L, ptr);
    
#if GLIB_MINOR_VERSION >= 10
    lua_pushboolean(L, constructor);
#else
    g_object_ref(ptr);

    if( GTK_IS_OBJECT(ptr) && GTK_OBJECT_FLOATING(ptr) )
    {
        lua_pushboolean(L, FALSE);
        gtk_object_sink(ptr);
    }
    else
    {
        lua_pushboolean(L, constructor);
    }
#endif
    lua_call(L, 2, 1);
}

static void struct_new(lua_State* L, gpointer ptr, gboolean need_unref)
{
    lua_pushliteral(L, "lgobStructNew");
    lua_rawget(L, LUA_REGISTRYINDEX);
    lua_pushlightuserdata(L, ptr);
    lua_pushboolean(L, need_unref);
    lua_call(L, 2, 1);
}

static void special_type_new(lua_State* L, const gchar* mt, gpointer ptr)
{
    if(ptr)
    {
        Object* obj = lua_newuserdata(L, sizeof(Object));
        obj->pointer = ptr;
        obj->need_unref = TRUE;
        lua_getfield(L, LUA_REGISTRYINDEX, mt);
        lua_setmetatable(L, -2);
    }
    else
        lua_pushnil(L);
}

static void copy_interface(lua_State* L, const char* iface, const char* target, const luaL_Reg* fnames)
{
    lua_getfield(L, -1, iface);
    lua_getfield(L, -2, target);
    
    for( ; fnames->name; ++fnames)
    {
        lua_pushstring(L, fnames->name);
        lua_rawget(L, -2);
        int clear = lua_isnil(L, -1);
        lua_pop(L, 1);
        
        if(clear) {
            lua_getfield(L, -2, fnames->name);
            lua_setfield(L, -2, fnames->name);
        }
    }
    
    lua_pop(L, 2);
}

static const struct luaL_Reg _st__global [];

static void priv_init_gtk(lua_State* L)
{
    int argc = 1;
    char** argv = g_malloc(argc * sizeof(char*));

    lua_getglobal(L, "arg");

    if(!lua_istable(L, -1))
        argv[0] = "lgob";
    else
    {
        lua_pushnumber(L, 0);
        lua_rawget(L, -2);
        argv[0] = (char*)luaL_optstring(L, -1, "lgob");
    }

    gtk_init(&argc, &argv);
    g_free(argv);
}

static int _wrap_gtk_version(lua_State* L)
{
    char rv[21], cv[21];

    snprintf(rv, 20, "%d.%d.%d", gtk_major_version, gtk_minor_version, gtk_micro_version);
    snprintf(cv, 20, "%d.%d.%d", GTK_MAJOR_VERSION, GTK_MINOR_VERSION, GTK_MICRO_VERSION);

    lua_pushstring(L, rv);
    lua_pushstring(L, cv);

    return 2;
}

static const struct luaL_Reg gtk [] =
{
    {"version", _wrap_gtk_version},
    {NULL, NULL}
};

static void _wrap_gtk_init(lua_State* L)
{
    priv_init_gtk(L);
    
    luaL_register(L, "gtk", gtk);
    luaL_register(L, NULL, _st__global);
    
    luaL_loadstring(L, "require('lgob.gobject')"); lua_call(L, 0, 0);
    luaL_loadstring(L, "gtk.Object = gobject.Object"); lua_call(L, 0, 0);
    luaL_loadstring(L, "gtk.InitiallyUnowned = gobject.Object"); lua_call(L, 0, 0);
    
    lua_getfield(L, LUA_REGISTRYINDEX, "lgobPrefix");
    lua_pushliteral(L, "Gtk"); lua_pushliteral(L, "gtk"); lua_rawset(L, -3);
    lua_pop(L, 1);
    
    luaL_loadstring(L, "glib.handle_log('Gtk')"); lua_call(L, 0, 0);
}

#include "proxylist.c"

static void _wrap_gtk_ret(lua_State* L)
{
    register_class(L, "ProxyList", "ListStore", proxylist);
    
    /* GtkBuilder */
    luaL_loadstring(L, "function gtk.Builder:connect_signals(h,d) if not h then h = _G end self:connect_signals_full(function(_,o,s,hn)o:connect(s,h[hn],d)end)end"); lua_call(L, 0, 0);
}

#include "caux.c"
