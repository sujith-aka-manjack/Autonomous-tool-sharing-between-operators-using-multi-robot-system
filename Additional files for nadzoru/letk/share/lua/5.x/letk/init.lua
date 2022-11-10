require 'letk.lua51to52'
require 'letk.table'
require 'letk.string'
require 'letk.math'

letk              = {}

function letk.varargsmap( fn, ... )
    local result = {}
    for n=1,select('#',...) do
        local e   = select(n,...)
        result[n] = fn( e, n )
    end
    return result
end

letk.trace = function( val, spaces, done )
    spaces = spaces or 0
    done   = done   or {}
    
    local pre = string.rep('    ',spaces)
    
    local t = type( val )
    if t == 'table' then
        local result = {}
        for k,v in pairs( val ) do
            if type(k) ~= 'table' then
                if type(v) ~= 'table' or not done[v] then
                    if type( v ) == 'table' then
                        done[v] = true
                    end
                    result[#result +1] = pre  .. '    [' .. letk.trace( k ) .. ']' .. " = " .. letk.trace( v, spaces+1, done )
                elseif type( v ) == 'table' then
                    result[#result +1] = pre  .. '    [' .. letk.trace( k ) .. ']' .. " = " .. tostring( v )
                end
            end
        end
        return "{ --" .. tostring(val) .. "\n" .. table.concat( result, ",\n" ) .. '\n' .. pre .. "}"
    elseif t == 'string' then
        return string.format("%q", val)
    else
        return tostring( val )
    end
end

letk.Class        = require'letk.class'
letk.List         = require'letk.list'
letk.Context      = require'letk.template.context'
letk.TemplateTags = require'letk.template.tags'
letk.Template     = require'letk.template'
letk.serialize    = require'letk.serialize'
letk.simpleio     = require'letk.simpleio'
letk.Cache        = require'letk.cache'
letk.debug        = require'letk.debug'
