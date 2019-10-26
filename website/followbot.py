from flask import Flask, render_template, redirect, url_for, request

app = Flask(__name__)

@app.route('/')
def homepage():
    return render_template('homepage.html')

@app.route('/process')
def process():
    return render_template('process.html')

@app.route('process/sprint-1')
def sprint1():
    return render_template('sprint1.html')

@app.route('process/sprint-2')
def sprint1():
    return render_template('sprint2.html')

@app.route('process/sprint-3')
def sprint1():
    return render_template('sprint3.html')

@app.route('process/final-sprint')
def finalsprint():
    return render_template('finalsprint.html')

@app.route('/systems')
def systems():
    return render_template('systems.html')

@app.route('systems/mechanical')
def mechanical():
    return render_template('mechanical.html')

@app.route('systems/electrical')
def electrical():
    return render_template('electrical.html')

@app.route('systems/software')
def software():
    return render_template('software.html')

@app.route('/budget')
def budget():
    return render_template('budget.html')

@app.route('/about-team')
def about():
    return render_template('aboutus.html')

if __name__ == '__main__':
    app.run()
