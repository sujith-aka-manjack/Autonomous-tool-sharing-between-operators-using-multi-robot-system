local function tag_if( template, chunk )
    local eval, err        = loadstring( 'return ' .. chunk[ 1 ] )
    if not( eval ) then 
        table.insert( template.erros, 'ERROR(if)' .. err )
        return false
    end
    local tlist, end_chunk = template:parse{ 'else', 'elseif', 'end', 'endif' }

    local flist

    if end_chunk.tag == 'else' then
        flist, end_chunk  = template:parse{ 'end', 'endif' }
    end

    if end_chunk.tag == 'elseif' then
        flist, end_chunk = tag_if( template, end_chunk )
    end

    return function( template, context )
        if context:eval( eval ) then
            return template:execute( tlist, context )
        else
            if type( flist ) == 'table' then
                return template:execute( flist, context )
            elseif type( flist ) == 'function' then
                return flist( template, context )
            end
            return ''
        end
    end
end

local function tag_print( template, chunk )
    if not chunk[ 1 ] or chunk[ 1 ] == '' then
        return
    end

    local eval, err = loadstring( 'return ' .. chunk[ 1 ] )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_print: ' .. err )
        return false
    end

    return function( template, context )
        local res =  context:eval( eval )
        return res
    end
end

local function tag_var( template, chunk )
    if not chunk.var or chunk.var == '' then
        return
    end

    local eval, err = loadstring( 'return ' .. chunk.var )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_var: ' .. err )
        return false
    end

    return function( template, context )
        local res =  context:eval( eval )
        return res
    end
end

local function tag_for( template, chunk )
    local lists      = {}
    local delimiters = { 'first', 'last', 'empty', 'loop', 'endfor', 'end', 'notlast', 'notfirst' }

    local list, end_chunk
    local last_end_chunk = 'loop'
    while true do
        list, end_chunk  = template:parse( delimiters )
        if not list or not end_chunk then
            print("ERRO: end for loop not find", chunk[1])
            return
        end
        lists[#lists +1] = {
            last_end_chunk,
            list,
        }

        last_end_chunk = end_chunk.tag

        if end_chunk.tag == 'end' or end_chunk.tag == 'endfor' then
            break
        end
    end

    --* arguments and expression *--
    local mode
    local arglist
    local explist

    if chunk[1]:match('^%s*(%w+)%s*=%s*.-%s*$' ) then
        --numeric for
        mode = 'numeric'
        arglist, explist = chunk[1]:match('^%s*([^%s=]+)%s*=%s*(.-)%s*$' )
        if not arglist then print("ERRO: invalid for", chunk[1]) end
    else
         --generic for
        mode = 'generic'
        arglist, explist = chunk[1]:match( '^%s*(.-)%s+in%s+(.-)%s*$' )
        if not arglist then print("ERRO: invalid for", chunk[1]) end
        arglist = string.split( arglist, ',' )
        for i, arg in ipairs( arglist ) do
            arglist[ i ] = string.trim( arg )
        end
    end

    local eval, err = loadstring( 'return ' .. explist )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_for: ' .. err )
        return false
    end

    return function( template, context )
        local for_ctx, result = {}, {}
        context:push( for_ctx, 'FOR' )

        local run             = false
        if mode == 'numeric' then
            local a,b,c = context:eval( eval )
            for i = a, b, c or 1 do
                run = true
                for_ctx[ arglist ] = i
                for _, lst in ipairs( lists ) do
                    if
                        lst[1] == 'loop' or
                        ( i == a                and lst[1] == 'first' ) or
                        ( i ~= a                and lst[1] == 'notfirst' ) or
                        ( i > ( b - (c or 1) )  and lst[1] == 'last'  ) or
                        ( i <= ( b - (c or 1) ) and lst[1] == 'notlast'  )
                    then
                        result[#result +1] = template:execute( lst[2], context )
                    end
                end
            end
        elseif mode == 'generic' then
            local iter, tbl, var  = context:eval( eval )
            local values = { iter( tbl, var ) }
            var = values[ 1 ]
            while var do
                local next_values = { iter( tbl, var ) }
                local next_var    = next_values[1]
                local islast      = next_var == nil

                for i, arg in ipairs( arglist ) do
                    for_ctx[ arg ] = values[ i ]
                end

                for _, lst in ipairs( lists ) do
                    if
                        (lst[1] == 'loop') or
                        ( not run        and lst[1] == 'first' ) or
                        ( run            and lst[1] == 'notfirst' ) or
                        ( islast         and lst[1] == 'last'  ) or
                        ( not islast     and lst[1] == 'notlast'  )
                    then
                        result[#result +1] = template:execute( lst[2], context )
                    end
                end

                values = next_values
                var    = next_var
                run = true
            end
        end
        if not run then
            for _, lst in ipairs( lists ) do
                if lst[1] == 'empty' then
                    result[#result +1] = template:execute( lst[2], context )
                end
            end
        end

        context:pop()

        return table.concat( result )
    end
end

--{% cycle 'a','b','c' as teste %}
local function tag_cycle( template, chunk )
    local external_key
    local explist = string.gsub( chunk[ 1 ], '%s+as%s+([%w_]+)%s*$', function( t )
        external_key = t
        return ''
    end )

    local itens, err = loadstring( 'return ' .. explist )
    if not( itens ) then 
        table.insert( template.errors, 'Error tag_cycle: ' .. err )
        return false
    end

    local iter
    return function( template, context )
        if iter == nil then
            iter = 0
        end
        local values = context:teval( itens )

        iter        = ( iter % #values ) + 1
        local value = values[ iter ]

        if external_key then
            local for_ctx = context:get_ctx('FOR')
            if for_ctx then
                for_ctx[ external_key ] = value
            end
        else
            return value
        end

        return ''
    end
end

local function tag_if_changed( template, chunk )
    local eval, err = loadstring( 'return ' .. chunk[ 1 ] )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_if_changed: ' .. err )
        return false
    end
    
    local tlist, end_chunk = template:parse{ 'else', 'end' }
    local flist
    if end_chunk.tag == 'else' then
        flist = template:parse{ 'end' }
    end
    local last_value
    return function( template, context )
        local new_value = context:eval( eval )
        if new_value ~= last_value then
            last_value = new_value
            return template:execute( tlist, context )
        elseif flist then
            return template:execute( flist, context )
        end
    end
end

local function tag_block( template, chunk )
    local name = string.match( chunk[ 1 ], '^%s*([%w_-]+)%s*$' )
    local list = template:parse{ 'end', 'endblock' }

    template.blocks[ name ] = list

    return function( template, context )
        return template:execute( template.blocks[ name ], context )
    end
end

local function tag_block_append( template, chunk )
    local name = string.match( chunk[ 1 ], '^%s*([%w_-]+)%s*$' )
    local list = template:parse{ 'end', 'endblock' }

    list.__APPEND           = true
    template.blocks[ name ] = list

    return function( template, context )
        return template:execute( template.blocks[ name ], context )
    end
end

local function tag_nl( template, chunk )
    local param = string.match( chunk[ 1 ], '^%s*(%d+)%s*$' )
    local n     = tonumber( param ) or 1

    return function( template, context )
        return string.rep('\n',n)
    end
end

local function tag_extends( template, chunk )
    local template_name         = chunk[ 1 ]
    local fn_template_name, err = loadstring( 'return ' .. template_name)
    if not( fn_template_name ) then 
        table.insert( template.errors, 'Error tag_extends: ' .. err )
        return false
    end

    --At this point only what are in {% block %} will be used by the template.blocks
    --~ local list_ignored = template:parse()
    local list_ignored, end_chunk = template:parse{ 'end', 'endextends' }

    return function( template, context )
        local template_name = context:eval( fn_template_name )
        local new_template  = template:sub_template( template_name )
        local status        = new_template:compile()
        if status then
            local list = new_template:parse()
            new_template:copy_blocks( template )
            return new_template:execute( list, context )
        end
        return new_template:getErrors()
    end
end


--[[ From the Django documentation:
  "The include tag should be considered as an implementation of “render this
subtemplate and include the HTML”, not as “parse this subtemplate and include
its contents as if it were part of the parent”. This means that there is no
shared state between included templates – each include is a completely
independent rendering process."
--]]
local function tag_include( template, chunk )
    local eval       = chunk[1]
    local template_name, with = eval:match( '^(.-)%s+with%s+(.+)%s*$' )
    if template_name and with then
        if with:match( '%S' ) then
            eval = template_name .. ', {' .. with .. '}'
        end
    end
    local f, err = loadstring( 'return ' .. eval )
    if not( f ) then 
        table.insert( template.errors, 'Error tag_include: ' .. err )
        return false
    end
    
    return function( template, context )
        local template_name, ctx = context:eval( f )
        local new_template       = template:sub_template( template_name )
        if type( ctx ) ~= 'table' then
            ctx = {}
        end
        context:push( ctx )
        local result = new_template( context )
        context:pop()
        return result
    end
end

local function tag_importblocks( template, chunk )
    local tmpl_eval       = chunk[1]
    
    local f, err = loadstring( 'return ' .. tmpl_eval )
    if not f then 
        table.insert( template.errors, 'Error tag_import_blocks: ' .. (err or '') )
        return false
    end

    local template_name = f()
    local new_template  = template:sub_template( template_name )

    new_template:compile_parse()
    template:copy_blocks( new_template )
    
    return function( template, context )
        return ''
    end
end

local function tag_with( template, chunk )
    local with, err = loadstring( 'return {' .. chunk[ 1 ] .. '}' )
    if not( with ) then 
        table.insert( template.errors, 'Error tag_with: ' .. err )
        return false
    end
    local list = template:parse{ 'end', 'endwith' }
    return function( template, context )
        local ctx = context:eval( with )
        context:push( ctx, 'WITH' )
        local result = template:execute( list, context )
        context:pop( )
        return result
    end
end

local function tag_load_filter( template, chunk )
    local filters, err = loadstring( 'return {' .. chunk[ 1 ] .. '}' )
    if not( filters ) then 
        table.insert( template.errors, 'Error tag_load_filter: ' .. err )
        return false
    end
    return function( template, context )
        local filter_names = context:eval( filters )
        for k_flt_nm, flt_nm in ipairs( filter_names ) do
            context:add_filter( flt_nm )
        end
        return ''
    end
end

local function tag_set( template, chunk )
    local f, err = loadstring( chunk[ 1 ] )
    if not( f ) then 
        table.insert( template.errors, 'Error tag_set: ' .. err )
        return false
    end
    
    return function( template, context )
        context:update( f )
    end
end

local templatetags = {
    ['openblock']     = '{%',
    ['closeblock']    = '%}',
    ['openvariable']  = '{{',
    ['closevariable'] = '}}',
    ['openbrace']     = '{',
    ['closebrace']    = '}',
    ['opencomment']   = '{#',
    ['closecomment']  = '#}',
}

local function tag_templatetag( template, chunk )
    local tagname = string.match( chunk[ 1 ], '^%s*(.-)%s*$' )
    if not( templatetags[ tagname ] ) then 
        table.insert( template.errors, 'Error tag_templatetag: tagname "' .. (tagname or '?') .. '" does not exist'  )
        return false
    end
    
    return function( template, context )
        return templatetags[ tagname ]
    end
end

local function tag_filter( template, chunk )
    local tmpl_eval = chunk[1]
    local f, err = loadstring( 'return ' .. tmpl_eval )
    if not f then 
        table.insert( template.errors, 'Error tag_filter: ' .. (err or '') )
        return false
    end
    local list = template:parse{ 'end', 'endfilter' }
    return function( template, context )
        local ctx = {}
        ctx.FILTER_STR = template:execute( list, context )
        context:push( ctx, 'FILTER' )
        local result = context:eval( f )
        context:pop( )
        return result
    end
end

local function tag_spaceless( template, chunk )
    local list = template:parse{ 'end', 'endspaceless' }
    return function( template, context )
        local result = template:execute( list, context )
        return result:gsub('>[%s\t\r\n]+<','><')
    end
end

local function tag_no_blank_line( template, chunk )
    local list = template:parse{ 'end', 'endnoblankline' }
    return function( template, context )
        local result = template:execute( list, context )
        return result:gsub('^[%s%t]*[\r\n]+','')
    end
end

--{% cache live=100,'name',UserID %} ... {% end cache %}
local function tag_cache( template, chunk )
    local eval, err        = loadstring( 'return {' .. chunk[ 1 ] .. '}' )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_cache: ' .. err )
        return false
    end
    local tlist, end_chunk = template:parse{ 'end', 'endcache' }
    
    return function( template, context )
        if not template._cache then
            return template:execute( tlist, context )    
        end
        
        local cache_info = context:eval( eval )
        local data       = template._cache:get( template.file, unpack(cache_info) )
        if not data then
            data = template:execute( tlist, context )
            template._cache:set( data, template.file, unpack(cache_info) )
        end
        if cache_info.live   then template._cache:live( cache_info.live, template.file, unpack(cache_info) ) end
        
        return data
    end
end

local function tag_cache_keys( template, chunk )
    local eval, err        = loadstring( 'return {' .. chunk[ 1 ] .. '}' )
    if not( eval ) then 
        table.insert( template.errors, 'Error tag_cache_keys: ' .. err )
        return false
    end
    
    return function( template, context )
        if not template._cache then
            return 'NOT CACHE AVAILABLE' 
        end
        
        local cache_info = context:eval( eval )
        return template._cache:keys( template.file, unpack(cache_info) )
    end
end

return {
    [ 'if' ]            = tag_if,
    [ 'print' ]         = tag_print,
    [ 'var' ]           = tag_var,
    [ 'for' ]           = tag_for,
    [ 'cycle' ]         = tag_cycle,
    [ 'ifchanged' ]     = tag_if_changed,
    [ 'block' ]         = tag_block,
    [ 'blockappend' ]   = tag_block_append,
    [ 'extends' ]       = tag_extends,
    [ 'include' ]       = tag_include,
    [ 'importblocks' ]  = tag_importblocks,
    [ 'with' ]          = tag_with,
    [ 'set' ]           = tag_set,
    [ 'nl' ]            = tag_nl,
    --~ [ 'autoescape' ]    = tag_autoescape, --Set a context Var and works in the tag_var
    [ 'load_filter' ]   = tag_load_filter,
    --~ [ 'csrf_token' ]    = tag_csrf_token,
    [ 'filter' ]        = tag_filter,
    ['templatetag']     = tag_templatetag,
    ['spaceless']       = tag_spaceless,
    ['noblankline']       = tag_no_blank_line,
    [ 'cache' ]         = tag_cache,
    [ 'cachekey' ]      = tag_cache_keys,
}

--[[
 verbatin
 debug
 firstof
 spaceless
 load (filters and tags)
 now
 regroup
 load_tags
--]]
