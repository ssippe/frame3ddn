## C# code
* fix warnings
* use var 
* prefer static classes 
* perfer static methods 
* use single assignment for local variables
* use lambda expressions instead of lambda queries
* use lambda expressions instead of for and foreach
* prefer to avoid deep nesting. use early return 
* prefer method body length under 40 lines. decompose in to sub-methods if required.
* when refactoring:
  * make the change,
  * build and fix errors, 
  * look for other code that is impacted by the change and change that.
* prefer to not use try/catch blocks
* use nameof or expressions as instead of hardcoding strings
  * e.g. `throw new NullReferenceException(nameof(user_metadata));`
