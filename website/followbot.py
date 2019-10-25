from flask import Flask, render_template, redirect, url_for, request

app = Flask(__name__)

@app.route('/')
def homepage:
    return render_template('homepage.html')

@app.route

if __name__ == '__main__':
    app.run()
