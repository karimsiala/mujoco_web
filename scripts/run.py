import re
import sys
from typing import List, Tuple, Optional
import clang.cindex


def configure_clang_library(libclang_path: str) -> None:
    """
    Configures the path to the libclang shared library.

    Args:
        libclang_path (str): The filesystem path to libclang-17.so.

    Raises:
        FileNotFoundError: If the specified libclang_path does not exist.
    """
    clang.cindex.Config.set_library_file(libclang_path)


def create_index() -> clang.cindex.Index:
    """
    Creates and returns a Clang index for parsing.

    Returns:
        clang.cindex.Index: An index object used to parse translation units.
    """
    return clang.cindex.Index.create()


def parse_header(
    index: clang.cindex.Index,
    header_path: str,
    include_dirs: Optional[List[str]] = None,
) -> clang.cindex.TranslationUnit:
    """
    Parses the given C++ header file and returns the translation unit.

    Args:
        index (clang.cindex.Index): The Clang index to use for parsing.
        header_path (str): The filesystem path to the C++ header file.
        include_dirs (Optional[List[str]]): A list of directories to include during parsing.

    Returns:
        clang.cindex.TranslationUnit: The parsed translation unit.

    Raises:
        clang.cindex.TranslationUnitLoadError: If parsing fails.
    """
    args = ["-x", "c++", "-std=c++17"]
    if include_dirs:
        for dir in include_dirs:
            args.extend(["-I", dir])

    return index.parse(header_path, args=args)


def find_class(
    node: clang.cindex.Cursor, class_name: str
) -> Optional[clang.cindex.Cursor]:
    """
    Recursively searches the AST to find the class declaration with the specified name.

    Args:
        node (clang.cindex.Cursor): The current AST node.
        class_name (str): The name of the class to find.

    Returns:
        Optional[clang.cindex.Cursor]: The cursor pointing to the class declaration, or None if not found.
    """
    if (
        node.kind
        in (clang.cindex.CursorKind.CLASS_DECL, clang.cindex.CursorKind.STRUCT_DECL)
        and node.spelling == class_name
    ):
        return node

    for child in node.get_children():
        result = find_class(child, class_name)
        if result:
            return result

    return None


def extract_members_with_comments(
    class_node: clang.cindex.Cursor, source_lines: List[str]
) -> List[Tuple[str, str, str]]:
    """
    Extracts member variables and their trailing comments from the given class node.

    Args:
        class_node (clang.cindex.Cursor): The cursor pointing to the class declaration.
        source_lines (List[str]): The lines of the source file.

    Returns:
        List[Tuple[str, str, str]]: A list of tuples containing member variable types, names, and comments.
    """
    members = []
    for child in class_node.get_children():
        if child.kind == clang.cindex.CursorKind.FIELD_DECL:
            var_type = child.type.spelling
            var_name = child.spelling

            # Get the line number where the variable is declared
            location = child.extent.end
            line_number = location.line - 1  # Zero-based index

            # Extract the line from the source file
            if 0 <= line_number < len(source_lines):
                line = source_lines[line_number]
                # Find the comment after the semicolon
                semicolon_index = line.find(";")
                if semicolon_index != -1:
                    comment = line[semicolon_index + 1 :].strip()
                    if comment.startswith("//"):
                        comment = comment[2:].strip()
                    elif comment.startswith("/*") and comment.endswith("*/"):
                        comment = comment[2:-2].strip()
                    else:
                        comment = ""
                else:
                    comment = ""
            else:
                comment = ""

            members.append((var_type, var_name, comment))
    return members


def extract_values_from_comment(comment: str) -> Optional[Tuple[str, str]]:
    """
    Extracts value1 and value2 from a comment string formatted as "(value1 x value2)".

    Args:
        comment (str): The comment string to parse.

    Returns:
        Optional[Tuple[str, str]]: A tuple containing (value1, value2) if the pattern is found; otherwise, None.
    """
    # Regular expression pattern to match "(value1 x value2)"
    pattern = r"\(([^()]+?)\s+x\s+([^)]+?)\)"

    # Search for the pattern in the comment
    match = re.search(pattern, comment)

    if match:
        # Extract and return the captured groups, stripping any surrounding whitespace
        value1 = match.group(1).strip()
        value2 = match.group(2).strip()
        return value1, value2

    # Return None if the pattern is not found
    return None


def generate_getters(members: List[Tuple[str, str, str]]) -> List[str]:
    """
    Generates getter method strings for the given member variables, including comments.

    If the member variable is a pointer type, the getter returns "val".
    Otherwise, it returns "m->variable_name".

    Args:
        members (List[Tuple[str, str, str]]): A list of tuples containing member variable types, names, and comments.

    Returns:
        List[str]: A list of formatted getter method strings with comments.
    """
    getters = []

    for var_type, var_name, comment in members:
        getter = ""
        value1 = ""
        value2 = ""

        parsed_comment = extract_values_from_comment(comment)
        if parsed_comment is not None:
            value1, value2 = parsed_comment
            # if "*" in value1:
            #     value1 = f"m->{value1}"
            # if "*" in value2:
            #     value2 = f"m->{value2}"

        if "*" in var_type:
            getter = f"val {var_name}() const {{ return val(typed_memory_view(m->{value1} * {value2}, m->{var_name})); }}"
        else:
            getter = f"{var_type} {var_name}() const {{ return m->{var_name}; }}"
        getters.append(getter)
    return getters


def main() -> None:
    """
    The main function orchestrates the parsing of the C++ header file and generates getter methods
    for the member variables of the mjModel_ class.
    """
    # Initialize Clang library.
    index = create_index()

    # Path to the C++ header file
    header_file = "/home/giovanni/repo/mujoco_web/build/external/mujoco-src/include/mujoco/mjmodel.h"

    # Include directories (modify as needed)
    include_dirs = ["/home/giovanni/repo/mujoco_web/build/external/mujoco-src/include"]

    # Read the source file lines
    try:
        with open(header_file, "r") as f:
            source_lines = f.readlines()
    except FileNotFoundError:
        print(f"Header file not found: {header_file}")
        sys.exit(1)

    # Parse the C++ header file
    try:
        translation_unit = parse_header(index, header_file, include_dirs)
    except clang.cindex.TranslationUnitLoadError as e:
        print(f"Error parsing header file: {e}")
        sys.exit(1)

    # Locate the model class
    model_class = find_class(translation_unit.cursor, "mjModel_")
    if not model_class:
        print("Class not found in the provided header file.")
        sys.exit(1)

    # Extract member variables
    member_vars = extract_members_with_comments(model_class, source_lines)
    if not member_vars:
        print("No member variables found in the class.")
        sys.exit(1)

    # Generate the code.
    getters = generate_getters(member_vars)
    for getter in getters:
        print(getter)


if __name__ == "__main__":
    main()
