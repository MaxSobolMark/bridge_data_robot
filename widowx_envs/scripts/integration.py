from Tic_Tac_Toe_Planner import Tic_Tac_Toe
from widowx_envs.utils.grasp_utils import rgb_to_robot_coords
from widowx_envs.utils.params import *

class Integrator:

    def __init__(self):
        self.board_bbox = np.array([])
        self.board_state = None
        self.t = Tic_Tac_Toe()

    def initialize_board_state(self, board_state, bbox, centroids):
        self.query = self.board_state != board_state
        self.board_state = board_state
        self.board_bbox = bbox
        self.centroids = centroids
        self.t.print_board(board_state)

    def query_LLM(self):
        if not self.query:
          return None
        else:
          move = self.t.get_LLM_move(self.board_state)
          self.t.print_board(self.board_state)
          return move

    def game_over(self):
        game_over_msg = self.t.game_over(self.board_state)
        if game_over_msg:
          print(game_over_msg)
          return True
        return False

    def get_robot_coords(self, move):
        top_left_x, top_left_y, width, height = self.board_bbox[0], self.board_bbox[1], self.board_bbox[2], self.board_bbox[3]
        x_coord = top_left_x + (width/6)*(move[1]*2 + 1)
        y_coord = top_left_y + (height/6)*(move[0]*2 + 1)
        center = np.array([x_coord, y_coord])
        return rgb_to_robot_coords(center, VILD_RGB_TO_ROBOT_TRANSMATRIX)