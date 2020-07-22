parent(john, nick).
parent(john, ann).
parent(helen, nick).
parent(helen, ann).
parent(nick, mary).
parent(nick, bill).
parent(margaret, mary).
parent(margaret, bill).
parent(mary, george).
parent(mary, jack).
parent(mary, alice).
parent(bill, peter).
parent(chris, paul).
parent(ann, paul).
parent(paul, sophie).

male(john).
male(nick).
male(bill).
male(george).
male(jack).
male(peter).
male(chris).
male(paul).

female(helen).
female(margaret).
female(mary).
female(alice).
female(ann).
female(sophie).

grandparent(X, Z) :- parent(X, Y), parent(Y, Z).
father(X, Y) :- male(X), parent(X, Y).
brother(X, Y) :- male(X), parent(Z, X), parent(Z, Y).

fixed_brother(X, Y) :- male(X), parent(Z, X), parent(Z, Y), different(X, Y).
different(X, Y) :- X \= Y.

mother(X, Y) :- female(X), parent(X, Y).
uncle(X, Y) :- parent(Z, Y), fixed_brother(X, Z).
sister(X, Y) :- female(X), parent(Z, X), parent(Z, Y), different(X, Y).
aunt(X, Y) :- parent(Z, Y), sister(X, Z).
grandchild(X, Y) :- grandparent(Y, X).
grandfather(X, Y) :- male(X), grandparent(X, Y).
grandmother(X, Y) :- female(X), grandparent(X, Y).
son(X, Y) :- male(X), parent(Y, X).
daughter(X, Y) :- female(X), parent(Y, X).
hbsad(X) :- son(_, X), daughter(_, X).

ancestor(X, Z) :- parent(X, Z).
ancestor(X, Z) :- parent(X, Y), ancestor(Y, Z).

