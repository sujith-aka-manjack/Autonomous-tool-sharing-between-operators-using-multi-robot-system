local Grammar = require 'letk.template.grammar'

local Template   = {}
Template.__index = Template

function Template.new( name )
    self = {}

    setmetatable( self, Template )

    self.name            = name
    self.blocks          = {}
    self.errors          = {}
    self.path            = {}
    self.remove_blank_lines = false
    self._cache          = nil
    self._escape         = nil -- function that escape context vars
    self.autoescape      = false -- by default does not escape

    return self
end

function Template:sub_template( name )
    local new_template = Template.new( name )
    
    new_template.errors             = self.errors
    new_template._cache             = self._cache
    new_template.path               = table.clone( self.path )
    new_template.remove_blank_lines = self.remove_blank_lines
    
    return new_template
end

function Template:copy_blocks( src_template )
    for k, blk in pairs( src_template.blocks ) do
        if blk.__APPEND then
            --The append will NOT copy the blk.__APPEND, which IS desired :)
            self.blocks[ k ] = table.append( self.blocks[ k ] or {}, blk )
        else
            self.blocks[ k ] = blk
        end
    end
end

function Template:compile_parse()
    if not self.chunks then
        local status = self:compile()
        if not status then
            return false, self:getErrors()
        end
    end

    return self:parse()
end

function Template:__call( context )
    local list = self:compile_parse()

    if list then
        return self:execute( list, context )
    else
        return self:getErrors()
    end
end

function Template:addError( str )
    self.errors[#self.errors + 1] = string.format( "ERROR [%s] :: %s", self.name or '<undefined>', str or '?' )
end

function Template:getErrors()
    return table.concat( self.errors, '\n' )
end

function Template:addPath( pathList )
    pathList = type( pathList ) == 'table' and pathList or { pathList }
    table.append( self.path, pathList )
end

function Template:getTemplateFile( name )
    local file
    for _, path in ipairs( self.path ) do
        file = io.open( path .. name, 'r' )
        if file then
            return file
        end
    end
    file = io.open( name, 'r' )
    if file then
        return file
    end
    return false
end 

function Template:setRemoveBlankLines( value )
    self.remove_blank_lines = value
end

function Template:cache( cache )
    self._cache = cache
end

function Template:compile()
    local f = self:getTemplateFile( self.name )
    if not f then
        --~ self.errors[#self.errors + 1] = string.format('File "%s" not found in "%s"',self.name, table.concat( self.path, ', '))
        self:addError( string.format( 'File not found in "%s"', table.concat( self.path, ', ') ) )
        return false
    end

    local s       = f:read("*a")

    self.chunks   = Grammar:match( s )
    self.chunk_id = 1
    
    return true
end

function Template:parse( fl )
    if type( fl ) == 'table' then
        for k,v in ipairs( fl ) do
            fl[v] = true
        end
    end

    local list = {}

    while true do
        local chunk = self.chunks[ self.chunk_id ]
        if not chunk then break end
        if type( chunk ) == 'string' then
            --~ chunk = chunk:gsub( '^\n', '' )
            --~ chunk = chunk:gsub('[ \t\r]*\n[ \t\r]*', '\n'):gsub('\n+', '\n'):gsub('^\n', '')
            if self.remove_blank_lines then
                --~ chunk = chunk:gsub( '^%s*[\n\r]*', '' )
            end
            chunk = { tag='print', [1] = string.format( '%q', chunk ) }
            self.chunks[ self.chunk_id ] = chunk
        elseif chunk.var then
            chunk.tag='var'
        end

        --DEBUG
        --print('[CHUNK]', chunk.tag, chunk.filter, chunk[1] )

        self.chunk_id = self.chunk_id + 1

        if fl and fl[ chunk.tag ] then
            return list, chunk
        end

        if chunk.tag then
            local tag = letk.TemplateTags[ chunk.tag ]
            if not tag then
                print( "Erro, invalid tag:", chunk.tag, chunk[1] )
            else
                local fn = tag( self, chunk )
                if fn then
                    list[#list +1] = fn
                end
            end
        end
    end

    return list
end

function Template:execute( list, context )
    if not list then return false, 'ERRO: no list' end
    if not context then return false, 'ERRO: no Context' end

    local result = {}
    for i, item in ipairs( list ) do
        local t = type( item )
        local val
        if t == 'string' then
            val = item
        elseif t == 'function' then
            val = item( self, context )
        else
            print('ERRO template execution')
        end
        result[ i ] = tostring( val or '' )
    end
    
    return table.concat( result, '' )
end

return Template
