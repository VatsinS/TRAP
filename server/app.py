from flask import Flask, render_template, jsonify, request
import numpy as np
import matplotlib.pyplot as plt
import json
import time

app = Flask(__name__)

@app.route('/')
def index():
    print("All good!")    
    return 200

@app.route('/api/', methods = ['POST','GET'])
def api():
    if request.method == "POST":
        sample_var = request.get_json(force=True)
        
        print(sample_var)
        sample_var = json.loads(sample_var)
        data = sample_var["base"]
        i = data["x"]
        y = data["y"]
        plt.scatter(int(i),int(y))
        plt.show()
        return "ok!"
    
    return "Plz use POST"

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

#plt.axis([0, 10, 0, 1])

"""
Sample code for plotting. 
Tested. Works.
Taken from: https://stackoverflow.com/questions/11874767/how-do-i-plot-in-real-time-in-a-while-loop-using-matplotlib
plt.axis([0, 10, 0, 1])
for i in range(10):
    y = np.random.random()
    plt.scatter(i, y)
    plt.pause(0.05)

plt.show()
"""

