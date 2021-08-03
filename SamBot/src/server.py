"""
server.py
Allows network control of SamBot via REST API.
Brandon Dunbar  Brandon.Dunbar97@gmail.com
"""

from flask import Flask
from flask_restful import Resource, Api, reqparse

from flaskResource import PublicInterface
from samBot import SamBot

app = Flask(__name__)
api = Api(app)
parser = reqparse.RequestParser()
parser.add_argument('token', type=str, help='API Token')
parser.add_argument('instruction', type=str, help='Instruction for the robot to execute')

api.add_resource(PublicInterface, '/', resource_class_kwargs={ 'parser': parser, 'samBot':  SamBot()})

if __name__ == '__main__':
    app.run(debug=True)

