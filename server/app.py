from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def index():
    print("All good!")    
    return "200"

