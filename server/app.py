from flask import Flask, render_template, jsonify, request

app = Flask(__name__)

@app.route('/')
def index():
    print("All good!")    
    return 200

@app.route('/api/', methods = ['POST','GET'])
def api():
    if request.method == "POST":
        sample_var = request.form["base"]
        return sample_var
    
    return 400

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')


