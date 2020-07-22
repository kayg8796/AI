% Implementation of depth first search
%
% The following predicates have to be defined:
% initial_state/1, final_state/1, move/2

dfs(States) :-
   initial_state(State),
   depth_first_search(State, [State], States).

depth_first_search(State, States, States) :-
   final_state(State).

depth_first_search(State1, SoFarStates, States) :-
   move(State1, State2),
   \+ member(State2, SoFarStates),    % To avoid cycles.
   append(SoFarStates, [State2], NewSoFarStates),
   depth_first_search(State2, NewSoFarStates, States).

% The farmer, wolf, goat and cabbage problem

initial_state(fwgc(left, left, left, left)).
final_state(fwgc(right, right, right, right)).

move(fwgc(F1, W1, G1, C1), fwgc(F2, W2, G2, C2)) :-
   opposite(F1, F2),
   ((W1 = W2, G1 = G2, C1 = C2) ;
    (opposite(W1, W2), G1 = G2, C1 = C2) ;
    (W1 = W2, opposite(G1, G2), C1 = C2) ;
    (W1 = W2, G1 = G2, opposite(C1, C2))),
   \+ illegal(fwgc(F2, W2, G2, C2)).

illegal(fwgc(F, W, G, C)) :-
   opposite(F, G),
   (G = W ; G = C).

opposite(left, right).
opposite(right, left).
