# Based on 
# - https://www.yukiyukiponsu.work/entry/python-Flask-btn-page-move
# - https://stackoverflow.com/a/49334973

from flask import Flask,render_template

app = Flask(__name__)

#「/」へアクセスがあった場合に、"Hello World"の文字列を返す
@app.route("/")
def hello():
    return "Hello World"

#「/templates」へアクセスがあった場合に、index.htmlを返す
@app.route("/templates", methods=["GET"])
def index():
    return render_template("index.html")

#「/nextpage」へアクセスがあった場合に、next_index.htmlを返す
@app.route("/nextpage", methods=["GET"])
def nextpage():
    return render_template("next_index.html")

#background process happening without any refreshing
@app.route('/background_process_test')
def background_process_test():
    print ("Hello")
    return ("nothing")

#app.pyをターミナルから直接呼び出した時だけ、app.run()を実行する
if __name__ == "__main__":
    app.run(debug=True)