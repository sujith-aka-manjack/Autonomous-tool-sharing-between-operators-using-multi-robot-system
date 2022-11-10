-- Taken from: http://lua-users.org/wiki/StringRecipes (Philippe Lhoste, Take Three)
function string.split( str, delim, maxNb )
    if string.find( str, delim ) == nil then
        return { str }
    end

    if maxNb == nil or maxNb < 1 then
        maxNb = 0    -- No limit
    end

    local result = { }
    local pat = "(.-)" .. delim .. "()"
    local nb = 0
    local lastPos
    for part, pos in string.gfind( str, pat ) do
        nb = nb + 1
        result[ nb ] = part
        lastPos = pos
        if nb == maxNb then break end
    end
    -- Handle the last field
    if nb ~= maxNb then
        result[ nb + 1 ] = string.sub( str, lastPos )
    end

    return result
end

function string.trim( s )
    return ( string.gsub( s, '^%s*(.-)%s*$', '%1' ) )
end

function string.nilifempty( s )
    return #s > 0 and s or nil
end
--

function string.expand( s, t )
    assert( type( s ) == 'string' )
    assert( type( t ) == 'table' )
    return ( string.gsub( s, '$(%w+)', t ) )
end

function string.interpolate( s, t )
    local function _gsub( n, k )
        local result
        if n == '#' then
            result = t[ tonumber( k ) ]
        else
            result = t[ k ]
        end
        --if type( result ) == 'string' then
            --result = string.gsub( result, '{(#?)([^}]+)}', _gsub )
        --end
        return result
    end
    local result = string.gsub( s, '{(#?)([^}]+)}', _gsub )
    return result
end

local _capitalize = function( a, b, c ) return string.upper( a ) .. string.lower( b ) end
function string.capitalize( s )
    assert( type( s ) == 'string' )
    if s == '' then return s end
    s = string.trim( string.gsub( s, ' +', ' ' ) )
    return string.sub( string.gsub( ' ' .. s, '(%w)(%w+)', _capitalize ), 2 )
end

local _htmltable = {
    ['<'] = '&lt;',
    ['>'] = '&gt;',
    ["'"] = '&#39;',
    ['"'] = '&quot;',
    ['&'] = '&amp;',
}
local _htmlpattern = '[' .. table.concat( table.keys( _htmltable ) ) .. ']'
function string.htmlencode( s )
    return (string.gsub( s or '', _htmlpattern, _htmltable ))
end

local xxunencodet = table.memoize( function( hex ) return string.char( tonumber( hex, 16 ) ) end )
local xxencodet = table.memoize( function( hex ) return string.format( '%%%02X', string.byte( c ) ) end )
xxencodet[ ' ' ] = '+'

function string.unescape( str )
    str = string.gsub( str, '%+', ' ' )
    str = string.gsub( str, '%%(%x%x)', xxunencodet )
    return str
end

function string.escape( str )
    str = string.gsub( str, '(%W)', xxencodet )
    return str
end

local _concat = {
    [ 1 ] = tostring,
    [ 2 ] = function( a, b )
        return tostring( a ) .. tostring( b )
    end,
    [ 3 ] = function( a, b, c )
        return tostring( a ) .. tostring( b ) .. tostring( c )
    end,
    [ 4 ] = function( a, b, c, d )
        return tostring( a ) .. tostring( b ) .. tostring( c ) .. tostring( d )
    end,
}
function string.tuple_concat( ... )
    local n = select( '#', ... )
    if n == 0 then return '' end
    if n <= 4 then return _concat[ n ]( ... ) end
    return string.tuple_concat( _concat[ 4 ]( ... ), select( 5, ... ) )
end

-- UTF8 handling
--~ local _capitalizeu8 = function( a, b, c ) return utf8.upper( a ) .. utf8.lower( b ) end
--~ utf8.capitalize = function( s )
    --~ if s == '' then return s end
    --~ s = string.trim( string.gsub( s, ' +', ' ' ) )
    --~ return string.sub( utf8.gsub( ' ' .. s, '(%w)(%w+)', _capitalizeu8 ), 2 )
--~ end

function string.abbr( str, num_chars, append_str )
    assert( type( str ) == 'string', "bad argument #1 to string.abbr (string expected, got " .. type( str ) .. ")" )
    assert( type( num_chars ) == 'number', "bad argument #2 to string.abbr (number expected, got " .. type( num_chars ) .. ")" )
    if append_str then
        assert( type( append_str ) == 'string', "bad OPTIONAL argument #3 to string.abbr (string expected, got " .. type( append_str ) .. ")" )
    end
    if #str <= (num_chars) then
        return str
    else
        return str:sub( 1, num_chars ) .. ( append_str or '' )
    end
end


