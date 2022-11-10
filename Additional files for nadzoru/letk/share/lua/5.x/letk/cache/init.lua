local Cache = {}

Cache.Base = letk.Class( function( self )

end )

function Cache.Base:set( data, ... )
	--Nothing
end

function Cache.Base:get( data, ... )
	return nil
end

function Cache.Base:check( data, ... )
	return false
end

function Cache.Base:live( data, ... )
	--Nothing
end

return Cache

