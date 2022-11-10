LUA_PKG = 'lua51'                      -- lua pkg-config file obs.: be sure to link with lua51.dll no liblua.a
LUA_EX  = 'lua5.1'                      -- lua executable
PKG     = 'i686-w64-mingw32-pkg-config' -- pkg-config program
EXT     = 'dll'                         -- shared library extension
CC      = 'i686-w64-mingw32-gcc'        -- C compiler
PWD     = 'pwd'                         -- current dir
RM      = 'rm -f'                       -- file removal
SHARED  = 'share/lua/5.1/lgob'          -- Lua shared dir
LIB     = 'lib/lua/5.1/lgob'            -- Lua lib dir
INST    = 'install -Dm644'              -- install a common file
INSTD   = 'install -dm755'              -- create a dir
SED     = 'sed'                         -- sed
CHMOD   = 'chmod +x'                    -- give executing permission
VERBOSE = false                         -- configure the volume of messages
DEBUG   = false                         -- compile with debug information

-- compiler flags
local opt     = DEBUG and '-g -O0 -DIDEBUG -DDEBUG' or '-Os'
COMPILE_FLAGS = string.format('%s -Wall -Wno-overflow -shared -Wl,--export-all', opt)
