# Some basics of python
## Creating Variables
* Unlike other programming languages, Python has no command for declaring a variable.
A variable is created the moment you first assign a value to it.
* Variables do not need to be declared with any particular type and can even change type after they have been set.
* A variable can have a short name (like x and y) or a more descriptive name (age, carname, total_volume). Rules for Python variables:
A variable name must start with a letter or the underscore character
A variable name cannot start with a number
A variable name can only contain alpha-numeric characters and underscores (A-z, 0-9, and _ )
Variable names are case-sensitive (age, Age and AGE are three different variables)
* The Python print statement is often used to output variables.
To combine both text and a variable, Python uses the + character
## Python numbers
* There are three numeric types in Python:
1. int
1. float
1. complex
Variables of numeric types are created when you assign a value to them.
To verify the type of any object in Python, use the type() function.
* Float can also be scientific numbers with an "e" to indicate the power of 10.
* Complex numbers are written with a "j" as the imaginary part:
## Strings
String literals in python are surrounded by either single quotation marks, or double quotation marks
'hello' is the same as "hello".
Strings can be output to screen using the print function. For example: print("hello").
* Python does not have a character data type, a single character is simply a string with a length of 1.
1. Get the character at position 1 (remember that the first character has the position 0)
1. Substring. Get the characters from position 2 to position 5 (not included):
1. The strip() method removes any whitespace from the beginning or the end:
1. The len() method returns the length of a string:
1. The lower() method returns the string in lower case:
1. The upper() method returns the string in upper case:
1. The replace() method replaces a string with another string:
1. The split() method splits the string into substrings if it finds instances of the separator:
## Python Collections (Arrays)

*    List is a collection which is ordered and changeable. Allows duplicate members.
 *   Tuple is a collection which is ordered and unchangeable. Allows duplicate members.
  *  Set is a collection which is unordered and unindexed. No duplicate members.
   * Dictionary is a collection which is unordered, changeable and indexed. No duplicate members.
### List
A list is a collection which is ordered and changeable. In Python lists are written with square brackets.
* You access the list items by referring to the index number:
* To change the value of a specific item, refer to the index number.
* You can loop through the list items by using a for loop.
* To determine if a specified item is present in a list use the in keyword:
* To determine how many items a list has, use the len() method:
* To add an item to the end of the list, use the append() method:
* To add an item at the specified index, use the insert() method:
* The remove() method removes the specified item:
* TheThe del keyword removes the specified index: pop() method removes the specified index, (or the last item if index is not specified
Python has a set of built-in methods that you can use on lists.
|command     | description                                                                   |<br />
|------------|-------------------------------------------------------------------------------|<br />
|append()    | add the element                                                               |<br />
|clear()     | removes the element                                                           |<br />
|copy()      |	Returns a copy of the list                                                   |<br />
|count()     |	Returns the number of elements with the specified value                      |<br />
|extend()    |	Add the elements of a list (or any iterable), to the end of the current list |<br />
|index()	   | Returns the index of the first element with the specified value               |<br />
|insert()    | Adds an element at the specified position                                     |<br />
|pop()       |	Removes the element at the specified position                                |<br />
|remove()    |	Removes the item with the specified value                                    |<br />
|reverse()   |	Reverses the order of the list                                               |<br />
|sort()      | Sorts the list                                                                |<br />
