local simpleio  = {}

function simpleio.printf( ... )
    print( string.format( ... ) )
end

function simpleio.sprintf( ... )
    return string.format( ... )
end

local scan_patterns = {
    ['s'] = { '(.+)'        ,          },
    ['i'] = { '(%d+)'       , tonumber },
    ['f'] = { '(%d*%.?%d*)' , tonumber },
    ['c'] = { '(.)'         ,          },
}

function simpleio.scanf( frm )
    local pos_processor = {}
    local num_patterns  = 0
    local pattern       = frm:gsub('[%(%)%.%+%-%*%?%[%]%^%$]', function( n )
        return '%' .. n
    end):gsub('%%([%a]*)', function( n )
        local sp = scan_patterns[ n ]
        if sp then
            num_patterns                = num_patterns + 1
            pos_processor[num_patterns] = sp[ 2 ]
            return sp[ 1 ]
        else
            return n
        end
    end)

    local str = io.read("*l")

    local results = { select(3, string.find( str, pattern ) ) }
    for i = 1, num_patterns do
        if pos_processor[ i ] then
            results[ i ] = pos_processor[ i ]( results[ i ] )
        end
    end

    return unpack( results )
end

return simpleio
