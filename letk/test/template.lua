require 'letk'
Template = letk.Template
Context  = letk.Context

obj = Template.new( './test.c' )
ctx = Context.new()
ctx:push{
    variavel  = "Ola, Tudo Bem?",
    html_var  = '<script> This "may" be a bug </script>',
    html_var2 = "<b> This need to be 'escaped' & tested </b>",
    num       = 12,
    cond1     = true,
    cond2     = false,
}
print( obj( ctx ) )
