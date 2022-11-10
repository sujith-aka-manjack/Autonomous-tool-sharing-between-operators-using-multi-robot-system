require 'redis'

letk.Cache.Redis = letk.Class( function( self, config )
	self.config          = config                or {}
	self.config.host     = self.config.host      or 'localhost'
	self.config.port     = self.config.port      or 6379
	self.config.database = self.config.database  or 0
	
	self.status, self.connection = pcall( Redis.connect, config )
	if not status then return nil end
	pcall( self.connection.select, self.connection, self.config.database )
end, letk.Cache.Base )

local function get_key( ... )
	return table.concat( {...}, '@' )
end

function letk.Cache.Redis:set( data, ... )
	local key = get_key( ... )
	return self.connection:set( key, data )
end

function letk.Cache.Redis:get( ... )
	local key = get_key( ... )
	return self.connection:get( key )
end

function letk.Cache.Redis:check( data, ... )
	local key = get_key( ... )
	return self.connection:exists( key )
end

function letk.Cache.Redis:live( seconds, ... )
	local key = get_key( ... )
	return self.connection:expire( key, seconds )
end

function letk.Cache.Redis:keys( ... )
	local key = get_key( ... ) .. '*'
	return table.concat( self.connection:keys( key ), '\n' )
end
