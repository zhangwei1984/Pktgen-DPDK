#ifndef _TEST_HELPERS_H
#define _TEST_HELPERS_H

/* Create a function that just returns. This is useful for generating function
 * definitions in the unit test files that are required by the linked
 * application code, but are not under test (and therefore don't have to do
 * anything useful).
 *
 * One of the specialized macros below can be used for convenience.
 *
 * Parameters:
 * - ret_type: return type of the function to generate
 * - func_name: name of the function to generate
 * - ret_value: value to return. Leave empty for void fuctions; use a valid
 *     value for a non-void ret_type.
 * - ...: parameter list of the function to generate
 */
#define STUB_FUNC(ret_type, func_name, ret_value, ...)                         \
    ret_type func_name(__VA_ARGS__) { return ret_value; }

#define STUB_VOID(func_name, ...)                                              \
    STUB_FUNC(void, func_name, , __VA_ARGS__)

#define STUB_NULL(ret_type, func_name, ...)                                    \
    STUB_FUNC(ret_type, func_name, NULL, __VA_ARGS__)


/*
 * Zero out a variable.
 */
#define ZERO(var) memset(&var, 0, sizeof(var));


#endif  // _TEST_HELPERS_H
