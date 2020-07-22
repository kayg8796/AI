position(atdoor).
position(middle).
position(atwindow).
position(corner1).
position(corner2).
position(corner3).
position(corner4).

move(state(P, onbox, B, hungry, athand), eat, state(P, onbox, B, fullup, instomach)) :-
   position(B),
   position(P).
move(state(middle, onbox, middle, E, ceiling), grasp, state(middle, onbox, middle, E, athand)).
move(state(P, onfloor, P, E, H), climb, state(P, onbox, P, E, H)) :-
   position(P).
move(state(P, onbox, P, E, H), getdown, state(P, onfloor, P, E, H)) :-
   position(P).
move(state(P1, onfloor, B, E, H), walk(P1, P2), state(P2, onfloor, B, E, H)) :-
   position(B),
   position(P1),
   position(P2),
   P1 \= P2.
move(state(P1, onfloor, P1, E, H), push(P1, P2), state(P2, onfloor, P2, E, H)) :-
   position(P1),
   position(P2),
   P1 \= P2.

canget(State, States, Moves) :-
   canget_iter(State, 1, States, Moves).

canget_iter(State, Lim, States, Moves) :-
   canget(State, Lim, [State], States, Moves),
   !.
canget_iter(State, Lim, States, Moves) :-
   Lim1 is Lim+1,
   canget_iter(State, Lim1, States, Moves).

canget(state(atdoor, _, atwindow, fullup, _), _, States, States, []).
canget(State1, Lim, SoFarStates, States, [Move|Moves]) :-
   Lim > 0,
   Lim1 is Lim-1,
   move(State1, Move, State2),
   \+ member(State2, SoFarStates),
   append(SoFarStates, [State2], NewSoFarStates),
   canget(State2, Lim1, NewSoFarStates, States, Moves).
