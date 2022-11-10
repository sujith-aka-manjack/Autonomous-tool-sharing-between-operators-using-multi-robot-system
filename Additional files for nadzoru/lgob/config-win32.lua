LUA_PKG = 'lua5.1'                  -- lua pkg-config file
LUA_EX  = 'lua'                     -- lua executable
PKG     = 'pkg-config'              -- pkg-config program
EXT     = 'dll'                     -- shared library extension
CC      = 'gcc'                     -- C compiler
PWD     = 'pwd'                     -- current dir
RM      = 'rm -f'                   -- file removal
SHARED  = ''                        -- Lua shared dir
LIB     = ''                        -- Lua lib dir
INST    = 'install -Dm644'          -- install a common file
INSTD   = 'install -dm755'          -- create a dir
SED     = 'sed'                     -- sed
CHMOD   = 'chmod +x'                -- give executing permission
VERBOSE = false                     -- configure the volume of messages

-- compiler flags
local opt = DEBUG and '-g -O0 -DIDEBUG' or '-Os'
COMPILE_FLAGS = string.format('%s -Wall -shared', opt)
