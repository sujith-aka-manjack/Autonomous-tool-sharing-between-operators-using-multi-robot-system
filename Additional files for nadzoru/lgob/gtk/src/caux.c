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

static gboolean visible_callback(GtkTreeModel *model, GtkTreeIter *iter, Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push the iter */
    struct_new(L, iter, FALSE);

    /* Call the function */
    lua_call(L, 2, 1);

    /* Get the return */
    gboolean res = lua_toboolean(L, -1);
    lua_settop(L, size);

    return res;
}

static void callback_free(gpointer user_data)
{
    Data* data = (Data*)user_data;

    /* Remove refs from Lua side */
    luaL_unref(data->L, LUA_REGISTRYINDEX, data->function_ref);
    luaL_unref(data->L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Free the data */
    g_free(data);
}

static int tree_path_handler(lua_State* L)
{
    GtkTreePath* path = lua_touserdata(L, -1);
    gchar* str = gtk_tree_path_to_string(path);
    lua_pushstring(L, str);
    g_free(str);
    
    return 1;
}

static gint sort_callback(GtkTreeModel* model, GtkTreeIter* a, 
    GtkTreeIter* b, Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push the iters */
    struct_new(L, a, FALSE);
    struct_new(L, b, FALSE);

    /* Call the function */
    lua_call(L, 3, 1);

    /* Get the return */
    gint res = lua_tointeger(L, -1);
    lua_settop(L, size);

    return res;
}

static void cell_layout_data_callback(GtkCellLayout* cell_layout,
    GtkCellRenderer *cell, GtkTreeModel *tree_model, GtkTreeIter *iter,
    Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push only the iter */
    struct_new(L, iter, FALSE);

    /* Call the function */
    lua_call(L, 2, 0);
    
    lua_settop(L, size);
}

static gboolean tree_foreach_callback(GtkTreeModel* model, GtkTreePath* path, 
    GtkTreeIter* iter, lua_State* L)
{
    int top = lua_gettop(L);
    
    /* Push the function and userdata */
    lua_pushvalue(L, -2);
    lua_pushvalue(L, -2);
    
    /* Push the iters */
    struct_new(L, path, FALSE);
    struct_new(L, iter, FALSE);

    /* Call the function */
    lua_call(L, 3, 1);

    /* Get the return */
    gboolean res = lua_toboolean(L, -1);
    
    lua_settop(L, top);

    return res;
}

static void builder_connect_callback(GtkBuilder* builder, GObject* object,
    const gchar* signal_name, const gchar* handler_name, GObject* connect_object,
    GConnectFlags flags, lua_State* L)
{
    int size = lua_gettop(L);
    
    /* By convention */ 
    lua_pushvalue(L, 2);    /* function */
    lua_pushvalue(L, 3);    /* userdata */

    /* Push the args */
    object_new(L, object,  FALSE);
    lua_pushstring(L, signal_name);
    lua_pushstring(L, handler_name);
    object_new(L, connect_object,  FALSE);
    lua_pushinteger(L, flags);
    
    /* Call the function */
    lua_call(L, 6, 0);
    lua_settop(L, size);
}

#define MAX_DRAG_TARGET_ENTRIES 20

/**
 * _fill_targets
 *
 * setup GtkTargetEntry array from Lua list
 *
 * target_list = {
 *   1 = { target = "STRING", info = 17, },
 *
 *   2 = 'text/plain',
 *
 *   3 = { target = "text/uri-list", info = 18,
 *     flags = gtk.TARGET_SAME_APP, },
 *   }
 *
 * @param L       thread
 * @param targets #GtkTargetEntry array to be filled
 * @param index   stack index of lua target list
 *
 * @return number of target entries
 */
static gint _fill_targets(lua_State *L, GtkTargetEntry *targets, gint index)
{
    gint n_targets=0;

    if (!lua_istable(L, index)) return(0);

    while (n_targets < MAX_DRAG_TARGET_ENTRIES)
    {
        GtkTargetEntry *entry = &targets[n_targets];

        lua_rawgeti(L, index, n_targets+1);  /* push corresponding list entry
        */

        if (lua_istable(L, -1))
        {
            lua_getfield(L, -1, "target");
            entry->target = (gchar *) lua_tostring(L, -1);
            lua_pop(L, 1);  /* pop target */

            lua_getfield(L, -1, "flags");
            entry->flags = lua_tointeger(L, -1);
            lua_pop(L, 1);  /* pop flags */

            lua_getfield(L, -1, "info");
            entry->info = lua_tointeger(L, -1);
            lua_pop(L, 1);  /* pop info */
        }
        else if (lua_isstring(L, -1))
        {
            entry->target = (gchar *) lua_tostring(L, -1);
            entry->flags = entry->info = 0;
        }
        else
        {
            lua_pop(L, 1);  /* pop invalid list entry */
            break;  /* at first invalid entry */
        }

        lua_pop(L, 1);  /* pop list entry */
        ++n_targets;
    }
    return(n_targets);
}

#if GTK_CHECK_VERSION(3,10,0)
static void update_header_callback(GtkListBoxRow *row, GtkListBoxRow *before, Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push the rows */
    object_new(L, row   , FALSE);
    object_new(L, before, FALSE);

    /* Call the function */
    lua_call(L, 3, 0);
    lua_settop(L, size);
}

static gboolean filter_header_callback(GtkListBoxRow *row, Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push the row */
    object_new(L, row, FALSE);

    /* Call the function */
    lua_call(L, 2, 1);
    gboolean res = lua_toboolean(L, -1);
    lua_settop(L, size);
    
    return res;
}

static gint sort_header_callback(GtkListBoxRow *row1, GtkListBoxRow *row2, Data* data)
{
    lua_State* L = data->L;
    int size = lua_gettop(L);

    /* Push the function and user data */
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->function_ref);
    lua_rawgeti(L, LUA_REGISTRYINDEX, data->ud_ref);

    /* Push the row */
    object_new(L, row1, FALSE);
    object_new(L, row2, FALSE);

    /* Call the function */
    lua_call(L, 3, 1);
    gint res = lua_tointeger(L, -1);
    lua_settop(L, size);
    
    return res;
}
#endif
