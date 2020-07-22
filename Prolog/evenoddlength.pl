evenlength([]).
evenlength([_|L]) :- oddlength(L).
oddlength([_|L]) :- evenlength(L).