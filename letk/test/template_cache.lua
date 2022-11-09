require 'letk'
require 'letk.cache.redis'
Template = letk.Template
Context  = letk.Context

cache = letk.Cache.Redis.new{ host='localhost', port=6379, database=0 }

obj = Template.new( './template_cache.html' )
obj:cache( cache )
ctx = Context.new()

ctx:push{
    cases1 = 5
}

print( obj( ctx ) )
