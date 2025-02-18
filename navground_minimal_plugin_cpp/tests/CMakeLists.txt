add_test(NAME discovery COMMAND navground info --behaviors Minimal)
set_property(TEST discovery PROPERTY PASS_REGULAR_EXPRESSION "Minimal")
