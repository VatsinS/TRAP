from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def index():    
    return "200"

@app.route('/api/')
def index():    
    return "200"


