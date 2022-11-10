LUA_PKG = 'lua5.1'                  -- lua pkg-config file
LUA_EX  = 'lua5.1'                  -- lua executable
PKG     = 'pkg-config'              -- pkg-config program
EXT     = 'so'                      -- shared library extension
CC      = 'gcc'                     -- C compiler
PWD     = 'pwd'                     -- current dir
RM      = 'rm -f'                   -- file removal
SHARED  = 'share/lua/5.1/lgob'      -- Lua shared dir
LIB     = 'lib/lua/5.1/lgob'        -- Lua lib dir
INST    = 'install -Dm644'          -- install a common file
INSTD   = 'install -dm755'          -- create a dir
SED     = 'sed'                     -- sed
CHMOD   = 'chmod +x'                -- give executing permission
VERBOSE = false                     -- configure the volume of messages
DEBUG   = false                     -- compile with debug information

-- compiler flags
local opt     = DEBUG and '-g -O0 -DIDEBUG -DDEBUG' or '-Os'
COMPILE_FLAGS = string.format('%s -Wall -Wno-overflow -shared -fPIC', opt)
