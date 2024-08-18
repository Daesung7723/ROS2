from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def hello():
    return 'hello'

@app.route('/1')
def test1page():
    return 'page 1'

@app.route('/2')
def test2page():
    return 'page 2'

@app.route('/ex')
def example():
    return render_template('example.html')

def main():
    app.run(debug=True, port=80)

if __name__ == '__main__' :
    main()