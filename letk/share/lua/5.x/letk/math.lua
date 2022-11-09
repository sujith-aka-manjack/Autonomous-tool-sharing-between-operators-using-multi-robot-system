local CHARS = {
    'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z',
    'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
    '0','1','2','3','4','5','6','7','8','9','_',
}
function math.random_string( lenght, chars )
    local t    = {}
    local c    = chars or CHARS
    local size = #c
    for i = 1, lenght do
        t[i] = c[ math.random(1,size) ]
    end
    return table.concat( t )
end

function math.div( a,b )
    return math.floor( a / b )
end

function math.normalise_deg( d )
    return d%360
end

--~ function math.deg_to_rad( d )
    --~ return (d/180) * math.pi
--~ end

--~ function math.rad_to_deg( r )
    --~ return (r/math.pi) * 180
--~ end

--from https://rosettacode.org/wiki/Averages/Mean_angle#Lua
function math.meanAngle( angleList )
    local sumSin, sumCos = 0, 0
    for i, angle in pairs( angleList ) do
        sumSin = sumSin + math.sin( math.rad( angle ) )
        sumCos = sumCos + math.cos( math.rad( angle ) )
    end
    local result = math.deg( math.atan2(sumSin, sumCos) )
    return result
end

function math.meanAngleFactor( angleList )
    local sumSin, sumCos = 0, 0
    for i, item in pairs( angleList ) do
        sumSin = sumSin + math.sin( math.rad( item[1] ) ) * item[2]
        sumCos = sumCos + math.cos( math.rad( item[1] ) ) * item[2]
    end
    local result = math.deg( math.atan2(sumSin, sumCos) )
    return result
end

function math.vec2DNormal( v )
    local x      = v.x or v[1]
    local y      = v.y or v[2]
    local length = math.sqrt( x^2 + y^2 )
    local nX = 0
    local nY = 0
    if length > 0.000001 then
        nX     = x/length
        nY     = y/length
    end
    
    return { nX, nY, x=nX, y=nY, length=length }
end

function math.round( x )
    return math.floor(x+0.5)
end
