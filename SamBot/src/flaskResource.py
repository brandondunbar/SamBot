"""
flaskResource.py
Holds the Resource class to be served to the Flask app.
Brandon Dunbar Brandon.Dunbar97@gmail.com
"""

from flask_restful import Resource

location = (0, 0)  # Dummy data, to be replaced when robot integration is ready


class PublicInterface(Resource):
    """Defines the public interface for the robot, including GET and POST request(s)."""

    def __init__(self, **kwargs):
        self.parser = kwargs['parser']
        self.bot = kwargs['samBot']

    def get(self):
        """Returns the current location held by the robot.

        Keyword arguments:
        token -- the API token necessary to complete request
        """
        args = self.parser.parse_args()
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
        args = self.parser.parse_args()
        if(not self._validateToken(args['token'])):
            return {'error': 'Invalid Token.'}
        elif('instruction' in args.keys() and self._validateInstruction(args['instruction'])):
            direction = int(args['instruction'].split()[-1])
            self.bot.creep(direction)
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


