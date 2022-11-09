local Context   = {}
Context.__index = Context

function Context.new( noBasePush )
    local self = {}
    setmetatable( self, Context )

    self.filter_MT = {
        __index = function( t, k )
            local t_MT = getmetatable( t )
            for k_flt, flt in ipairs( t_MT ) do
                if flt[k] then return flt[k] end
            end
        end,
        require 'letk.template.filters',
    }
    local filter    =  setmetatable( {}, self.filter_MT )

    self.ctxs      = {}
    self.ctxs_type = {}
    if not noBasePush then
        self:push{
            table    = table.clone( table ),
            string   = table.clone( string ),
            math     = table.clone( math ),
            filter   = filter,
            tonumber = tonumber,
            tostring = tostring,
            select   = select,
            pairs    = pairs,
            ipairs   = ipairs,
            type     = type,
        }
    end

    return self
end

function Context:add_filter( path )
    local flt = require( path )
    table.insert( self.filter_MT, flt )
end

function Context:push( t, name )
    if type( t ) == 'table' then
        self.ctxs[ #self.ctxs + 1 ]       = t
        self.ctxs_type[ #self.ctxs_type + 1 ] = name or '?'
    end
end

function Context:pop()
    if #self.ctxs > 0 then
        self.ctxs[ #self.ctxs ]           = nil
        self.ctxs_type[ #self.ctxs_type ] = nil
    end
end

function Context:get( k )
    for i = #self.ctxs, 1, -1 do
        local v = self.ctxs[ i ][ k ]
        if v ~= nil then
            return v
        end
    end
end

function Context:set( k, v, ctx_type )
    local ctx = self.ctxs[ #self.ctxs ]
    ctx[ k ]  = v
end

function Context:get_ctx( ctx_type )
    for i = #self.ctxs, 1, -1 do
        if self.ctxs_type[ i ] == ctx_type then
            return self.ctxs[ i ]
        end
    end
end

function Context:update( f )
        local env = setmetatable( {}, {
            __index    = function( _, k ) return self:get( k )  end,
            __newindex = function( _, k, v )
                for i = #self.ctxs, 1, -1 do
                    if self.ctxs[ i ][ k ] then
                        self.ctxs[ i ][ k ] = v
                        return
                    end
                end
                self.ctxs[ #self.ctxs ][ k ] = v
            end,
        } )
        local old_env = getfenv( f )
        setfenv( f, env )
        f()
        setfenv( f, old_env )
end


function Context:get_env()
    local env = setmetatable( {}, {
        __index = function( _, k ) return self:get( k ) end,
        __newindex = error,
    } )
    return env
end

function Context:teval( f )
    local old_env = getfenv( f )
    local env     = self:get_env()
    setfenv( f, env )
    local result  = { f() }
    setfenv( f, old_env )
    return result
end

function Context:eval( f )
    local result = self:teval( f )
    return unpack( result )
end



return Context
