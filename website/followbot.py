from flask import Flask, render_template, redirect, url_for, request

app = Flask(__name__)

@app.route('/')
def homepage():
    return render_template('homepage.html')

@app.route('/about-team')
def about():
    return render_template('aboutus.html')

@app.route('/mechanical')
def mechanical():
    return render_template('mechanical.html')

@app.route('/electrical')
def electrical():
    return render_template('electrical.html')

@app.route('/software')
def software():
    return render_template('software.html')

@app.route('/process')
def process():
    return render_template('process.html')

if __name__ == '__main__':
    app.run()
