"""main.py

Main python file for SamBot API endpoint(s), yet to be integrated.

Brandon Dunbar
brandon.dunbar97@gmail.com
"""

from flask import Flask
from flask_restful import Resource, Api, reqparse
# from _ import RobotInterface  # TODO: Create RobotInterface class

location = (0, 0)  # Dummy data, to be replaced when robot integration is ready

app = Flask(__name__)
api = Api(app)
parser = reqparse.RequestParser()
parser.add_argument('token', type=str, help='API Token')
parser.add_argument('instruction', type=str, help='Instruction for the robot to execute')


class PublicInterface(Resource):
    """Defines the public interface for the robot, including GET and POST request(s)."""

    robotInterface = None  # new RobotInterface()

    def get(self):
        """Returns the current location held by the robot.

        Keyword arguments:
        token -- the API token necessary to complete request
        """
        args = parser.parse_args()
        if(self._validateToken(args['token'])):
            return {'location': str(location)}
        else:
            return {'error': 'Invalid Token.'}
    
    def post(self):
        """Passes the provided instruction to the robot after validating it.

        Keyword arguments:
        token -- the API token necessary to complete request
        instruction -- the instruction for the robot to execute; currently none available
        """
        args = parser.parse_args()
        if(not self._validateToken(args['token'])):
            return {'error': 'Invalid Token.'}
        elif('instruction' in args.keys() and self._validateInstruction(args['instruction'])):
            return {'valid': True}
        else:
            return {'valid': False}
    
    def _validateToken(self, tokenToValidate):
        """Validates the provided token."""
        if(tokenToValidate):
            return True
        else:
            return False
    
    def _validateInstruction(self, instructionToValidate):
        """Validates the provided instruction."""
        if(instructionToValidate):
            return True
        else:
            return False


api.add_resource(PublicInterface, '/')

if __name__ == '__main__':
    app.run(debug=True)