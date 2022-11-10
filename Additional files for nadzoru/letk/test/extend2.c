This is Extend2.c

{% block a %}
    Este é o bloco 'a' do extend2.c
    {% if true %}
        SIM SIM
    {% else %}
        NÃO NÃO
    {% end %}
    A var é {{ variavel }}
{% endblock %}

{% block c %}
    Este é o bloco 'c' do extend2.c
{% endblock %}
