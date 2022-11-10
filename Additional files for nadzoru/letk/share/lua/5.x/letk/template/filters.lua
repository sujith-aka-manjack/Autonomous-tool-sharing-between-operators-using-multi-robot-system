local filter = {}

function filter.addslashes( str )
    return str:gsub('\'', '\\\'')
end

function filter.default( str, def )
    if not str then return def end
    return str
end


local units = {'', 'K', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y'}
function filter.filesizeformat( num, decimal, bits, precision )
    precision = tonumber( precision ) or 2
    num       = tonumber( num )       or 0
    local div = decimal and 1000 or 1024

    local unit = 1
    while num > (div-1) do
        unit = unit + 1
        num  = num / div
    end

    local bin_dec_text  = ''
    if not decimal and unit > 1 then
        bin_dec_text = 'i'
    end
    local bit_byte_text = bits    and 'b' or 'B'

    num = string.format( "%." .. precision .. "f", num ):gsub('0*$',''):gsub('%.$','')

    return string.format("%s %s%s%s", num, units[unit], bin_dec_text, bit_byte_text )
    
end

function filter.clear_empty_lines( str )
    return str:gsub('\n[%s\t]*\n','\n')
end

return filter
