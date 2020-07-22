my_member(X, [X|Tail]).
my_member(X, [Head|Tail]) :-
   my_member(X, Tail).

my_append([], L, L).
my_append([X|L1], L2, [X|L3]) :-
   my_append(L1, L2, L3).

langford(L) :- 
   L = [_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_],
   append(_, [9,_,_,_,_,_,_,_,_,_,9,_,_,_,_,_,_,_,_,_,9|_], L),
   append(_, [8,_,_,_,_,_,_,_,_,8,_,_,_,_,_,_,_,_,8|_], L),
   append(_, [7,_,_,_,_,_,_,_,7,_,_,_,_,_,_,_,7|_], L),
   append(_, [6,_,_,_,_,_,_,6,_,_,_,_,_,_,6|_], L),
   append(_, [5,_,_,_,_,_,5,_,_,_,_,_,5|_], L),
   append(_, [4,_,_,_,_,4,_,_,_,_,4|_], L),
   append(_, [3,_,_,_,3,_,_,_,3|_], L),
   append(_, [2,_,_,2,_,_,2|_], L),
   append(_, [1,_,1,_,1|_], L).

my_delete(X, [X|Tail], Tail).
my_delete(X, [Y|Tail], [Y|Tail1]) :-
   my_delete(X, Tail, Tail1).

insert(X, List, BiggerList) :-
   my_delete(X, BiggerList, List).

sublist(S, L) :-
   append(L1, L2, L),
   append(S, L3, L2).

permutation([], []).
permutation([X|L], P) :-
   permutation(L, L1),
   insert(X, L1, P).

permutation2([], []).
permutation2(L, [X|P]) :-
   my_delete(X, L, L1),
   permutation2(L1, P).

my_reverse([], []).
my_reverse([First|Rest], Reversed) :-
   my_reverse(Rest, ReversedRest),
   append(ReversedRest, [First], Reversed).

reverse1(List1, List2) :-
   rev_app(List1, [], List2).

rev_app([], List, List).
rev_app([X|List1], List2, List3) :-
   rev_app(List1, [X|List2], List3).

my_flatten([], []).
my_flatten([L1|L2], FL) :-
   my_flatten(L1, FL1),
   my_flatten(L2, FL2),
   append(FL1, FL2, FL).
my_flatten(X, [X]).

fixed_flatten([], []) :-
   !.
fixed_flatten([L1|L2], FL) :-
   !,
   fixed_flatten(L1, FL1),
   fixed_flatten(L2, FL2),
   append(FL1, FL2, FL).
fixed_flatten(X, [X]).
