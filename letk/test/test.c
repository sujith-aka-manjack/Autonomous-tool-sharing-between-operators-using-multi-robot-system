//Teste 1
[1]Primeira Linha: OK

//Teste 2
{% if true %}
[2] if true: OK
{% else %}
[2] if true: ERRO
{% end %}

//Teste 3
{% if false %}
[3] if false: ERRO
{% else %}
[3] if false: OK
{% end %}

//Teste 4
{% if 1==1 %}
[4] if 1==1: OK
{% elseif 2==2 %}
[4] if elseif a: ERRO
{% elseif 3==3 %}
[4] if elseif b: ERRO
{% else %}
[4] if else: ERRO
{% end %}

//Teste 5
{% if 1==2 %}
[5] if 1==1: ERRO
{% elseif 2==2 %}
[5] if elseif a: OK
{% elseif 3==3 %}
[5] if elseif b: ERRO
{% else %}
[5] if else: ERRO
{% end %}

//Teste 6
{% if false %}
[6] if 1==1: ERRO
{% elseif false %}
[6] if elseif a: ERRO
{% elseif 3==3 %}
[6] if elseif b: OK
{% else %}
[6] if else: ERRO
{% end %}

//Teste 7
{% if false %}
[7] if 1==1: ERRO
{% elseif false %}
[7] if elseif a: ERRO
{% elseif 3~=3 %}
[7] if elseif b: ERRO
{% else %}
[7] if else: OK
{% end %}

//Teste 8
{% if true %}
[8] if: OK
{% endif %}

//Teste 9
{% if false %}
[9] if: ERRO
{% endif %}
[9] if only this: OK

//Teste 10
[10] Minha Variável {{ variavel }}

//Teste 11
[11] Minha Variável {{ string.upper(variavel) }}

//Teste 12
[12] Minha Variável {{ num }} == 12

//Teste 13
[13] Minha Variável {{ num + 2 }} == 14

//Teste 14
{% for i=1,10,2 %}
    Val {{i}}
{% first %}
    This is the first {{ i }}
{% notfirst %}
    This is NOT the first {{ i }}
{% last %}
    This is the last {{ i }}
{% empty %}
    It is Empty: ERRO
{% notlast %}
    This is NOT the last {{ i }}
{% loop %}
    ENDING loop {{ i }} -------------------
{% end %}

//Teste 15
{% for c,v in ipairs{'a','b','c','d'} %}
    {{ c }} = {{ v }}
{% first %}
    This is the first {{ c }} = {{ v }}
{% notfirst %}
    This is NOT the first {{ c }} = {{ v }}
{% last %}
    This is the last {{ c }} = {{ v }}
{% empty %}
    It is Empty: ERRO
{% notlast %}
    This is NOT the last {{ c }} = {{ v }}
{% loop %}
    ENDING loop {{ c }} = {{ v }} -------------------
{% end %}

//Teste 16
{% for c,v in ipairs{} %}
    {{v}}
{% empty %}
    Empty: OK
{% endfor %}

//Teste 17
{% for c,v in pairs{a = '111', b='222'} %}
    {{ c }} = {{ v }}
{% first %}
    This is the first {{ tostring(c) .. '-' .. v }}
{% last %}
    This is the last {{ v }}
{% empty %}
    It is Empty: ERRO
{% loop %}
    Back to loop {{ v }}
{% notlast %}
    This is NOT the last {{ c }} = {{ v }}
{% end %}

//Teste 18
{% for i=1,3 %}
    --{{i}}
    {% for j=4,6 %}
        {{i}},{{j}}
        {% for c,v in pairs{'a','b'} %}
            {{v}}
        {% end %}
    {% end %}
{% end %}

//Teste 19 a
{% for i=1,10 %}
    {{i}}{% cycle 'a','b',i as teste %}, cycle: {{teste}}
{% end %}

//Teste 19 b
{% for i=1,10 %}
    {{i}}--{% cycle 'a','b',i,2*i %}
{% end %}

//Teste 20 a
{% for c,v in ipairs{'a','a','b','b','c'} %}
    {% ifchanged v %}
        --> {{ v }}
    {% else %}
        repeat: {{ v }}
    {% end %}
{% end %}

//Teste 20 b
{% for c,v in ipairs{'a','a','b','b','c'} %}
    {% ifchanged v %}
        --> {{ v }}
    {% end %}
{% end %}

//Teste 21
{% include 'extend.c' %}

//Teste 22
{% with x = 33, z=42 %}
    {% for i=0,10 %}
        {{x * i}}--{{z+i}}
    {% end %}
{% end %}

//Teste 23
{% with x = 12; y = { a = 13, b = 14 } %}
    {{x}}, {{ y.a }}, {{ y.b }}
    {% set x = 15; y.a = 16 %}
    {{x}}, {{ y.a }}, {{ y.b }}
{% end %}

//Teste 24
{% with x = 12; y = { a = 13, b = 14 } %}
    {{x}}, {{ y.a }}, {{ y.b }}
    {% for i = 1,3 %}
        {% set x = i; y.a = y.a*i %}
        {{x}}, {{ y.a }}, {{ y.b }}
    {% end %}
    {{x}}, {{ y.a }}, {{ y.b }}
{% end %}

//Teste 25
This is the tag {% templatetag openblock %} templatetag {% templatetag closeblock %}
Your can print a variable using {% templatetag openvariable %} var_name {% templatetag closevariable %} 
Coments are made using {% templatetag opencomment %} comment {% templatetag closecomment %}
Also {% templatetag openbrace %} and {% templatetag closebrace %} are available

//Teste 26
[25] Minha Variável sem filtro {{ html_var }}
[25] Minha Variável com filtro {{ string.htmlencode( html_var ) }}
[25] Minha Variável 2 com filtro {{ string.htmlencode( html_var2 ) }}

An file with {{ filter.filesizeformat(1023) }} - 1023 B

An file with {{ filter.filesizeformat(1024) }} - 1 KiB
An file with {{ filter.filesizeformat(1025) }} - 1 KiB

An file with {{ filter.filesizeformat(1048575) }} - 1 MiB
An file with {{ filter.filesizeformat(1048576) }} - 1 MiB
An file with {{ filter.filesizeformat(123456789) }} - 117.74 MiB

An file with {{ filter.filesizeformat(1073741824) }} - 1 GiB
An file with {{ filter.filesizeformat(1524713390) }} - 1.42 GiB

{% filter filter.clear_empty_lines( FILTER_STR ) %}

    //Teste 27
    {% filter string.htmlencode( FILTER_STR ) %}
        <html>
            <head>Using block filter & badass</head>
            <body>
                This "is" an example 'of' block filter
            </body>
        </html>
    {% end %}

    //Teste 28
    {% load_filter 'filter_test' %}

    {{ filter.test( 'bla' ) }}

{% endfilter %}

//Test 29
{% spaceless %}
    <p> 
        <a href="foo/">Bar</a>
        <a href="foo/">Bar</a>
    </p>
{% end %}

