require 'letk'
Template = letk.Template
Context  = letk.Context

obj = Template.new( 'extend.c' )
ctx = Context.new()
ctx:push{
    variavel = 'TESTE',
}
print( obj( ctx ) )
