import os, copy, sys
import numpy as np


def start_page():
    print('REVERSI BOARD GAME')
    print('1: New game.\n2 :Quit game.')
    opt = int(input('Select choice: '))
    if opt == 1:
        depth = 4
        n, depthStr, colour = input('Enter board size, level(DEFAULT: 4) and colour(white/black): ').split()
        if depthStr != '': depth = int(depth)
        if colour == 'white':
            pr = 0
        elif colour == 'black':
            pr = 1
        else:
            print('invalid input for colour')
        return int(n), depth, pr
    elif opt == 2:
        sys.exit(0)
    else:
        print('invalid input try again')
        start_page()


n, depth, pr = start_page()
alpha = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
         'w', 'x', 'y', 'z']
board = [[' ' for x in range(n)] for y in range(n)]
# 8 directions
dirx = [-1, 0, 1, -1, 1, -1, 0, 1]
diry = [-1, -1, -1, 0, 0, 1, 1, 1]

def InitBoard():
    #board = [[' ' for x in range(n)] for y in range(n)]
    if n % 2 == 0: # if board size is even
        z = int((n - 2) / 2)
        board[z][z] = 'W'
        board[n - 1 - z][z] = 'B'
        board[z][n - 1 - z] = 'B'
        board[n - 1 - z][n - 1 - z] = 'W'


def PrintBoard(b):
    # This function prints out the board that it was passed. Returns None.
    HLINE = '  '+n*'+---'+'+'
    print(end=' ')
    for i in range(n):
        print('   %s' % (alpha[i]),end='')
    print('')
    print(HLINE)
    for y in range(n):
        print(y + 1, end=' ')
        for x in range(n):
            print('| %s' % (b[x][y]), end=' ')
        print('|')
        print(HLINE)

def MakeMove(board1, x, y, player): # assuming valid move
    totctr = 0 # total number of opponent pieces taken
    board1[y][x] = player
    for d in range(8): # 8 directions
        ctr = 0
        for i in range(n):
            dx = x + dirx[d] * (i + 1)
            dy = y + diry[d] * (i + 1)
            if dx < 0 or dx > n - 1 or dy < 0 or dy > n - 1:
                ctr = 0
                break
            elif board1[dy][dx] == player:
                break
            elif board1[dy][dx] == ' ':
                ctr = 0
                break
            elif board1[dy][dx] == '*':
                ctr = 0
                break
            else:
                ctr += 1
        for i in range(ctr):
            dx = x + dirx[d] * (i + 1)
            dy = y + diry[d] * (i + 1)
            board1[dy][dx] = player
        totctr += ctr
    return (board1, totctr)

def ValidMove(board1, x, y, player):
    if x < 0 or x > n - 1 or y < 0 or y > n - 1:
        return False
    if board1[y][x] != ' ':
        return False
    #print('got here\n')
    (boardTemp, totctr) = MakeMove(copy.deepcopy(board1), x, y, player)
    if totctr == 0:
        return False
    return True

minEvalBoard = -1 # min - 1 , usually negative infinity
maxEvalBoard = n * n + 4 * n + 4 + 1 # max + 1 , usually should be infinity
def EvalBoard(board, player):
    tot = 0
    for y in range(n):
        for x in range(n):
            if board[y][x] == player:
                if (x == 0 or x == n - 1) and (y == 0 or y == n - 1):
                    tot += 4 # corner
                elif (x == 0 or x == n - 1) or (y == 0 or y == n - 1):
                    tot += 2 # side
                else:
                    tot += 1 # central
    return tot

# if no valid move(s) possible then True
def IsTerminalNode(board, player):
    for y in range(n):
        for x in range(n): # the valid move
            if ValidMove(board, x, y, player):
                #print('check\n')
                return False
    return True


def AlphaBeta(board, player, depth, alpha, beta, maximizingPlayer):
    if depth == 0 or IsTerminalNode(board, player):
        return EvalBoard(board, player)
    if maximizingPlayer:
        v = minEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = max(v, AlphaBeta(boardTemp, player, depth - 1, alpha, beta, False))
                    alpha = max(alpha, v)
                    if beta <= alpha:
                        break # beta cut-off
        return v
    else: # minimizingPlayer
        v = maxEvalBoard
        for y in range(n):
            for x in range(n):
                if ValidMove(board, x, y, player):
                    (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                    v = min(v, AlphaBeta(boardTemp, player, depth - 1, alpha, beta, True))
                    beta = min(beta, v)
                    if beta <= alpha:
                        break # alpha cut-off
        return v


def count(b):
    for i in range(n):
        wh = 0
        bl = 0
        for i in range(n):
            for j in range(n):
                if b[i][j] == 'W':
                    wh += 1
                elif b[i][j] == 'B':
                    bl += 1
        return (wh, bl)


def show_move(player, b, p):
    for i in range(n):
        for j in range(n):
            # PrintBoard(b)
            # (boardTemp1, totctr1) = MakeMove(copy.deepcopy(board), i, j, player)
            if ValidMove(b, j, i, player) and b[i][j] == ' ':
                # PrintBoard(b)
                # print('{} and {} true'.format(i,j))
                # if totctr1 > 1 and board[i][j] == ' ':
                b[i][j] = '*'
    if p != pr:
        PrintBoard(b)
    for i in range(n):
        for j in range(n):
            if board[i][j] == '*':
                board[i][j] = ' '


def BestMove(board, player):
    maxPoints = 0
    mx = -1
    my = -1
    for y in range(n):
        for x in range(n):
            if ValidMove(board, x, y, player):
                (boardTemp, totctr) = MakeMove(copy.deepcopy(board), x, y, player)
                points = AlphaBeta(board, player, depth, minEvalBoard, maxEvalBoard, True)

                if points > maxPoints:
                    maxPoints = points
                    mx = x
                    my = y
    return (mx, my)


players = ('B', 'W')
InitBoard()
PrintBoard(board)
while True:
    for p in range(2):
        print('')
        # PrintBoard()
        show_move(players[1 - pr], board, p)
        player = players[p]
        # show_move(player,board)
        (wh, bl) = count(board)
        print('Score black: ' + str(bl))
        print('Score white  : ' + str(wh))
        print('PLAYER: ' + player)

        if IsTerminalNode(board, players[0]) and IsTerminalNode(board, players[1]):
            print('Player cannot play! Game ended!')
            if bl > wh:
                print('black player Wins!')
            elif bl < wh:
                print('white player wins!')
            else:
                print('Game is a tie!')

            sys.exit(0)
        if p == pr:  # computer's turn
            (x, y) = BestMove(board, player)
            if not (x == -1 and y == -1):
                (board, totctr) = MakeMove(board, x, y, player)
                print('computer plays: ' + alpha[y] + str(x + 1))
                # print ('# of pieces taken: ' + str(totctr))
                # show_move(players[1-pr],board,p)



        else:  # user's turn
            # show_move(player,board)
            while True:
                if IsTerminalNode(board, player):
                    break
                y, x = input('Human plays: ')
                x = int(x) - 1
                y = alpha.index(y)
                # print('here\n')
                if ValidMove(board, x, y, player):
                    (board, totctr) = MakeMove(board, x, y, player)
                    PrintBoard(board)
                    # print ('# of pieces taken: ' + str(totctr))
                    break

                else:
                    print('Invalid move! Try again!')
