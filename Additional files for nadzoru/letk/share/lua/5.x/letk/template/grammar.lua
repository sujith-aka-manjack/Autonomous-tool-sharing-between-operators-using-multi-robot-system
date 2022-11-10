local lpeg = require 'lpeg'

--Patters
local V = lpeg.V --Variable (non-terminal)
local P = lpeg.P --Matches string literally or exactly number characters
local S = lpeg.S --Set
local R = lpeg.R --Range

--Capture
local C  = lpeg.C  --Simple Captures
local Ct = lpeg.Ct --Table Capture
local Cc = lpeg.Cc --Constant Capture
local Cg = lpeg.Cg --Group Capture

local SS = V'IGNORED'^0
local Sp = V'IGNORED'^1

--path^1 = path+, path^0 = path*, path^-1 = path?

local Grammar = {
    "Start", -- initial
    Start    = Ct( ( V'Literal' + V'Tag' )^ 0 ),
    Tag      = Ct(
        V'Comment' +
        --~ (S' '^0 * V'BlockTag' * S' '^0 * S'\n') +
        V'BlockTag' +
        V'ExprTag'
    ),
    Literal  = C( ( 1 - V'Tag' )^1 ),
    Comment  = P'{#' * ( 1 - P'#}' )^0 * P'#}',
    BlockTag = P'{%' * SS * Cg (V'Ident', "tag" ) *
                       Sp * C( ( 1 - P'%}' ) ^ 0 ) *
                       SS * P'%}',
                       
    ExprTag  = P'{{' * Cg( ( 1 - P'}}' )^0, "var" ) * P'}}',

    --~ ExprTag  = P'{{' * Cg( ( 1 - P'}}' - V'Filter' )^0, "var" ) * V'Filter'^-1 * P'}}',
    --~ Filter   = P'|' * SS * Cg (V'Ident', "filter" ) * SS,

    --~ ExprTag  = P'{{' * SS * V'ExprVar' * SS * V'Filters'^-1 * SS * P'}}',
    --~ ExprVar  = Cg( ( 1 - P'}}' - V'Filters' )^0, "var" ),
    --~ Filters  = Cg( Ct( V'Filter'^0 ), "filters" ),
    --~ Filter   = P'|' * SS * Cg (V'Ident', "name" ) * SS * ( P':' * C(1 - P'|' - P'}}') )^0  * SS,

    Ident    = R( 'az', 'AZ', '__' ) * R( 'az', 'AZ', '09', '__' ) ^ 0,
    IGNORED  = S' \t\r\n\f',
}

return P( Grammar )
