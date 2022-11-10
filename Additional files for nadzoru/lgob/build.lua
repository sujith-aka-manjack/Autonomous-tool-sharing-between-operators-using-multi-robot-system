#! /usr/bin/env lua5.1

require('config')

-- functions

local sf = string.format

function interpolate(s, tab)
    if not tab then tab = _G end
    return (s:gsub('($%b{})', function(w)
        local p = w:sub(3, -2)
        return tab[p] or _G[p] or error('var ' .. w .. ' not found') end
    ))
end

function ex(...)
    local cmd = interpolate(...)
    
    if VERBOSE then
        print(cmd)
    end
    
    local f = io.popen(cmd)
    local a = f:read('*a')
    f:close()
    a = a:gsub('\n$', '')
    return a
end

function init(mod)
    print(interpolate('** Building module ${name} **', mod))
end

function mkdir(cmd)
    ex('${INSTD} ' .. cmd)
end

function chmod(cmd)
    ex('${CHMOD} ' .. cmd)
end

function pkg(tbl, arg)
    local t = {arg = arg or '--cflags --libs'}
    return ex('${PKG} ${arg} ' .. table.concat(tbl, ' '), t)
end

function generate(mod)
    local t = {
        name    = mod.name,
        input   = interpolate('${MOD}/src/${name}.ovr', mod),
        output  = interpolate('${MOD}/src/iface.c'         ),
        log     = interpolate('${MOD}/src/log'             ),
        version = pkg({mod.pkg}, '--modversion'            ),
    }
    
    ex('${GEN} -i ${input} -o ${output} -l ${log} -v ${version}', t)
end

function compile(mod)
    if not mod.src then mod.src = 'iface.c' end

    local t = {
        flags  = pkg{LUA_PKG, mod.pkg},
        input  = interpolate('${MOD}/src/${src}'        , mod),
        output = interpolate('${MOD}/src/${name}.${EXT}', mod),
    }
    
    ex('${CC} ${COMPILE_FLAGS} -I${DEST}/include ${input} -o ${output} ${flags}', t)
end

function install(arg)
    if type(arg) == 'string' then
        ex('${INST} ' .. arg)
    else
        ex('${INST} ${MOD}/src/${name}.${EXT} ${DEST}/${LIB}/${name}.${EXT}', arg)
    end
end

function clean(mod)
    local garbage = {'iface.c', 'log', interpolate('${name}.${EXT}', mod)}

    for i = 1, #garbage do
        ex('${RM} ${MOD}/src/' .. garbage[i])
    end
end

-- ** Main **

local usage = 'Usage: ./build.lua module dest'
MOD  = assert(arg[1], usage)
DEST = assert(arg[2], usage)
GEN  = interpolate('${LUA_EX} ${DEST}/bin/lgob-generator')

require(interpolate('${MOD}/mod'))
