local function serialize ( val )
    local t = type( val )
    if t == 'table' then
        local result = {}
        for k,v in pairs( val ) do
            if type(k) ~= 'table' then
                result[#result +1] = '[' .. serialize( k ) .. ']' .. " = " .. serialize( v )
            end
        end
        return "{\n" .. table.concat( result, ",\n" ) .. "}"
    elseif t == 'string' then
        return string.format("%q", val)
    else
        return tostring( val )
    end
end

return serialize

