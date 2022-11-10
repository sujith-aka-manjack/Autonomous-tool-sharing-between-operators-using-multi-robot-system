/*
 * gcc -pipe -shared -fpic -Wall `pkg-config --cflags --libs lua5.1` linuxserial.c -o linuxserial.so
 * 
 */

/* Lua headers */
#include <lua.h>
#include <lauxlib.h>

/* C headers*/
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define LIB_NAME "letk.linuxserial"
#define LIB_VERSION "0.1"

typedef struct SSerialFile
{
    int filedesc;
    struct termios options;
} SerialFile;

int version_letk_linuxserial(lua_State* L);
int luaopen_letk_linuxserial(lua_State *L);

int LSopen(lua_State *L){
    luaL_checktype(L, 1, LUA_TSTRING);

    int fd = open( lua_tostring(L,1) , O_RDWR | O_NOCTTY | O_NDELAY );
    if( fd != -1 ){
        SerialFile *f = (SerialFile *) lua_newuserdata( L, sizeof(SerialFile) );
        luaL_getmetatable(L, "serialMT");
        lua_setmetatable(L, -2);
        f->filedesc = fd;
        
        //~ f->filedesc = open( lua_tostring(L,1) , O_RDWR | O_NOCTTY | O_NDELAY );
        tcgetattr( f->filedesc, &f->options );
        /* f->options.c_cflag &= ~( ICANON | ECHO | ECHOE |ISIG ); *//* Set non-canonical mod, no echo, ... */
        f->options.c_cflag &= ~ICANON;
        f->options.c_cflag &= ~ECHO;
        f->options.c_cc[VTIME] = 5; /* Set timeout of 0.5 seconds */
        cfsetispeed( &f->options, B9600 );
        cfsetospeed( &f->options, B9600 );
        /* f->options.c_cflag |= ( CLOCAL | CREAD ); */
        /* f->options.c_cflag &= ~PARENB; */
        /* f->options.c_cflag &= ~CSTOPB; */
        /* f->options.c_cflag &= ~CSIZE; */
        /* f->options.c_cflag |= CS8; */
        /* f->options.c_iflag &= ~( IXON | IXOFF | IXANY ); */
        /* f->options.c_oflag &= ~OPOST; */
        tcsetattr( f->filedesc, TCSANOW, &f->options );
    } else {
        lua_pushnil( L );
    }
    
    return 1;
}

int LSwrite(lua_State *L){
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TSTRING);
    SerialFile *f   = (SerialFile *) lua_touserdata(L, 1);
    const char *buf = lua_tostring(L,2);
    write( f->filedesc, buf, strlen( buf ) );
    
    return 0;
}

int LSwriteByte(lua_State *L){
    luaL_checktype(L, 1, LUA_TUSERDATA);
    luaL_checktype(L, 2, LUA_TNUMBER);
    SerialFile *f   = (SerialFile *) lua_touserdata(L, 1);
    char buf[2];
    sprintf( buf, "%c", (int) lua_tointeger(L, 2) );
    write( f->filedesc, buf, strlen( buf ) );
    
    return 0;
}

int LSread(lua_State *L){
    char buf[256];
    int readBytes;
    luaL_checktype(L, 1, LUA_TUSERDATA);
    SerialFile *f   = (SerialFile *) lua_touserdata(L, 1);
    readBytes = read( f->filedesc, buf, 256 );
    if( readBytes > 0 ){
        buf[ readBytes ] = '\0';
        lua_pushstring( L, buf );
    } else {
        lua_pushnil( L );
    }
    lua_pushinteger( L, readBytes );
    return 2;
}

int LSreadByte(lua_State *L){
    char buf[1];
    int readBytes;
    luaL_checktype(L, 1, LUA_TUSERDATA);
    SerialFile *f   = (SerialFile *) lua_touserdata(L, 1);
    readBytes = read( f->filedesc, buf, 1 );
    if( readBytes > 0 ){
        lua_pushinteger( L, buf[0] );
    } else {
        lua_pushnil( L );
    }
    return 1;
}

static const struct luaL_reg LSserial [] =
{
    {"version"   , version_letk_linuxserial },
    {"open"      , LSopen                   },
    {"write"     , LSwrite                  },
    {"writeByte" , LSwriteByte              },
    {"read"      , LSread                   },
    {"readByte" , LSreadByte              },
    {NULL, NULL}
};

int version_letk_linuxserial(lua_State* L)
{
    lua_pushstring(L, "letk.linuxserial 0.1");
    
    return 1;
}

//~ int luaopen_letk_linuxserial(lua_State *L){
int luaopen_linuxserial(lua_State *L){
    luaL_register(L, "serial", LSserial); /*1 (1)*/
    luaL_newmetatable(L, "serialMT"); /*2 (1,2)*/
    lua_pushvalue(L, -2); /*3(cp 1[serial]) (1,2,3)*/
    lua_setfield(L, -2, "__index"); /*2[__index] = 3 (1,2)*/
    lua_pop(L, 1); /*(1)*/
    lua_pop(L, 1); /*()*/
    
    return 1;
}
