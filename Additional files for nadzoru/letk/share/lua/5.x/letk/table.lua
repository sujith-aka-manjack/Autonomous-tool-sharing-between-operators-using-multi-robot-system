function table.keys( t )
    local r = {}
    local i = 1
    for k in pairs( t ) do
        r[ i ] = k
        i = i + 1
    end
    return r
end

function table.values( t )
    local r = {}
    local i = 1
    for _, v in pairs( t ) do
        r[ i ] = v
        i = i + 1
    end
    return r
end

--------------------------------------------------------------------------------

local function spairs_iter( t, key )
    local id = 1
    if key then
        id = t.__ord[ key ] + 1
    end
    local k = t.__keys[ id ]
    return k, t.__src[ k ]
end

--Sorted pairs
function spairs( src )
    local t = {}
    t.__src  = src
    t.__keys = table.keys( src )
    table.sort( t.__keys  )
    t.__ord = table.transpose( t.__keys  )
    
    return spairs_iter, t, nil
end

--------------------------------------------------------------------------------

function table.count( t )
    local i = 0
    for _ in pairs( t ) do
        i = i + 1
    end
    return i
end

function table.icount( t )
    local i = 0
    for _ in ipairs( t ) do
        i = i + 1
    end
    return i
end

function table.complete( dst, src )
    if src then
        for k, v in pairs( src ) do
            if dst[ k ] == nil then
                dst[ k ] = v
            end
        end
    end
    return dst
end

function table.complete_deep( dst, src )
    if src then
        for k, v in pairs( src ) do
            if dst[ k ] == nil then
                if type( v ) == 'table' then
                    dst[ k ] = table.clone( v, true )
                else
                    dst[ k ] = v
                end
            elseif type( dst[ k ] ) == 'table' and type( v ) == 'table' then
                table.complete_deep( dst[ k ], v )
            end
        end
    end
    return dst
end

--

function table.transpose( t, value )
    local r = {}
    if value ~=  nil then
        for _, v in pairs( t ) do
            r[ v ] = value
        end
    else
        for k, v in pairs( t ) do
            r[ v ] = k
        end
    end

    return r
end

--transpose only integers, copy others
function table.itranspose( t, value )
    local r    = {}
    local used = {}

    --transpose integers
    for k, v in ipairs( t ) do
        if value ~=  nil then
            r[ v ] = value
        else
            r[ v ] = k
        end
        used[ k ] = true
    end

    --keep others
    for k, v in pairs( t ) do
        if not used[ k ] then
            r[ k ] = v
        end
    end

    return r
end

function table.iproject( t, key, dst_key )
    local result = {}
    if dst_key then
        for i, row in ipairs( t ) do
            result[ row[ key ] ] = row[ dst_key ]
        end
    else
        for _, row in ipairs( t ) do
            result[ row[ key ] ] = row
        end
    end
    return result
end

function table.project( t, key )
    local result = {}
    for _, row in pairs( t ) do
        result[ row[ key ] ] = row
    end
    return result
end

function table.makeset( t )
    return table.transpose( t, true )
end

function table.makeiset( t )
    return table.itranspose( t, true )
end

function table.clone( t, deep )
    local r = {}
    for k, v in pairs( t ) do
        if deep and type( v ) == 'table' then
            r[ k ] = table.clone( v, true )
        else
            r[ k ] = v
        end
    end
    return r
end

function table.memoize( f )
    local t = {}
    setmetatable( t, {
        __index = function( self, k )
            rawset( self, k, f( k ) )
            return rawget( self, k )
        end } )
    return t
end

function table.map( t, fn_map )
    local result = {}
    local j = 0
    for i, value in ipairs( t ) do
        local new_value = fn_map( value )
        if new_value ~= nil then
            j = j + 1
            result[ j ] = new_value
        end
    end
    return result
end

function table.append( dst, src, ... )
    local n = #dst
    if src then
        for i, val in ipairs( src ) do
            dst[ n + i ] = val
        end
    end
    if ... then return table.append( dst, ... ) end
    return dst
end

--~ function table.prepend( dst, src, ... )
    --~ local n    = #src
    --~ local temp = {}
    --~ for i, val in ipairs( dst ) do
        --~ temp[i] = val
    --~ end
    --~ for i, val in ipairs( src ) do
        --~ dst[i] = val
    --~ end
    --~ for i, val in ipairs( temp ) do
        --~ dst[i+n] = val
    --~ end
    --~ if ... then return table.prepend( dst, ... ) end
    --~ return dst
--~ end

function table.update( dst, src, ... )
    if src then
        for key, value in pairs( src ) do
            dst[ key ] = value
        end
    end
    
    if ... then return table.update( dst, ... ) end
    
    return dst
end

function table.update_deep( dst, src, addNumberKey )
    if src then
        for key, value in pairs( src ) do
            if addNumberKey and type( key ) == 'number' then
                if type( value ) == 'table' then
                    dst[ #dst + 1 ] = table.clone( value, true )
                else
                    dst[ #dst + 1 ] = value
                end
            else
                if type( value ) == 'table' then
                    dst[ key ] = type( dst[ key ] ) == 'table' and dst[ key ] or {}
                    table.update_deep( dst[ key ], value )
                else
                    dst[ key ] = value
                end
            end
        end
    end
    return dst
end

function table.pluck( tab, key )
    local result = {}
    for i, row in ipairs( tab ) do
        result[ i ] = row[ key ]
    end
    return result
end

function table.slice( t, start, finish )
    start = start or 1
    finish = finish or #t
    local result = table.pack( table.unpack( t, start, finish ) )
    return result
end

function table.find_key( t, value )
    for k, v in pairs( t ) do
        if v == value then
            return k
        end
    end
end

function table.remove_value( t, value )
    for i, v in ipairs( t ) do
        if v == value then
            table.remove( t, i )
            return
        end
    end
end

function table.inject( t, src0, srcn, dst0, dstn, ... )
    local a = src0 or 1
    local b = srcn or select( '#', ... )
    local c = dst0 or 1
    local d = dstn or #t

    -- put in order
    if d < c then c, d = d, c end
    if b < a then a, b = b, a end

    local s = src0
    for i = c, d do
        t[ i ] = select( src0, ... )
        src0 = src0 + 1
    end

    -- TODO: check if the ranges are differents
    local n_src = b - a
    local n_dst = d - c
    if n_src < n_dst then
        for i = d, d - ( n_dst - n_src ) + 1, -1 do
            table.remove( t, i )
        end
    end
    return t
end

function table.generate( f, i, n, s )
    local result = {}
    if i and n then
        s = s or 1
        for iter = i, n, s do
            result[ iter ] = f( iter )
        end
    else
        i = 0
        while true do
            i = i + 1
            local v = f( i )
            if v == nil then break end
            result[ i ] = v
        end
    end
    return result
end

function table.to_int_array( str )
    if type( str ) == 'table' then
        local result = {}
        for i, v in ipairs( str ) do
            v = tonumber( v )
            if not v then return end
            result[ i ] = v
        end
        return result
    elseif type( str ) == 'string' then
        if not str:match( '^%s*([%d,]+)%s*$' ) then
            return
        end
        local result = {}
        for int in str:gmatch( '(%d+)' ) do
            result[ #result + 1 ] = tonumber( int )
        end
        return result
    end
end

function table.concatkv( tbl, sep1, sep2 )
    local t = {}
    for k,v in pairs( tbl ) do
        table.insert(t, string.format( '%s%s%s', tostring(k), tostring( sep2 or ''), tostring(v) ) )
    end
    return table.concat( t, sep1 )
end

function table.tostringconcat( tbl, sep, i, j )
    local t = {}
    for k,v in ipairs( tbl ) do
        t[k] = tostring( v )
    end
    return table.concat( t, sep, i, j  )
end

function table.sort_field( tbl, fld )
    table.sort( tbl, function(a, b)
        local aa = a[ fld ]
        local bb = b[ fld ]
        if aa == nil and bb == nil then return false end
        if aa ~= nil and bb ~= nil then return aa < bb end
        return bb == nil
    end)
    return tbl
end



