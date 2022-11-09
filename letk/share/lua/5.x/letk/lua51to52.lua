setfenv = setfenv or function(f, t)
    f = (type(f) == 'function' and f or debug.getinfo(f + 1, 'f').func)
    local name
    local up = 0
    repeat
        up = up + 1
        name = debug.getupvalue(f, up)
    until name == '_ENV' or name == nil
    if name then
      debug.upvaluejoin(f, up, function() return t end, 1) -- use unique upvalue, set it to f
    end
end

getfenv = getfenv or function(f)
    f = (type(f) == 'function' and f or debug.getinfo(f + 1, 'f').func)
    local name, env
    local up = 0
    repeat
        up = up + 1
        name, env = debug.getupvalue(f, up)
    until name == '_ENV' or name == nil
    return env
end

loadstring = loadstring or load
unpack     = unpack or table.unpack
