local letkDebug = {}

function letkDebug.checkType( variable, ... )
    local t = type( variable )

    for k_typeName, typeName in ipairs{ ... } do
        if t == typeName then
            return true, t
        end
    end

    return false, t
end

function letkDebug.assertType( variable, ... )
    local status, typeName = letkDebug.checkType( variable, ... )
    return assert( status, "type " .. table.concat( {...}, ", " ) .. " expected, got " .. typeName )
end


return letkDebug
