solve_jugs(JugC1, JugC2, JugF, States) :-
   InitialState = jugs(0,0),
   dfs(JugC1, JugC2, JugF, InitialState, [InitialState], States).

dfs(_, _, JugF, jugs(0,JugF), States, States).
dfs(JugC1, JugC2, JugF, State1, SoFarStates, States) :-
   move(JugC1, JugC2, State1, State2),
   \+ member(State2, SoFarStates),
   append(SoFarStates, [State2], NewSoFarStates),
   dfs(JugC1, JugC2, JugF, State2, NewSoFarStates, States).

move(JugC1, JugC2, jugs(V1, V2), jugs(W1, W2)) :-
   L is V1+V2,
   ((V1 < JugC1, W1 = JugC1, W2 = V2) ;      % Fill up jug 1
    (V2 < JugC2, W1 = V1, W2 = JugC2) ;      % Fill up jug 2
    (V1 > 0, W1 = 0, W2 = V2) ;              % Empty jug 1
    (V2 > 0, W1 = V1, W2 = 0) ;              % Empty jug 2
    (minimax(L, JugC2, W2), W1 is L-W2) ;    % Pour water from jug 1 to jug 2
    (minimax(L, JugC1, W1), W2 is L-W1)).    % Pour water from jug 2 to jug 1

minimax(X, Y, X) :- X =< Y.
minimax(X, Y, Y) :- X > Y.
