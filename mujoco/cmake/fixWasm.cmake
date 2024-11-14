# This function is a workaround to fix the following issue which arises when
# Emscripten and Vite are used together:
#
# [vite:worker-import-meta-url] Vite is unable to parse the worker options as
# the value is not static.To ignore this error, please use /* @vite-ignore */
# in the worker options.
# 
# The issue is describer in detail here:
# https://github.com/emscripten-core/emscripten/issues/22394
#
# The fix uses some regular expressions to modify the generated WASM .js file.

function(fixWasm filepath)

    message("Updating file: ${filepath}...")

    # Read the content of the file
    file(READ "${filepath}" content)
    
    # Remove the `workerOptions` declaration from content
    string(REGEX REPLACE "var[ \t\r\n]+workerOptions[ \t\r\n]*=[ \t\r\n]*{[ \t\r\n]*[^}]*};" "" content "${content}")

    # Replace all occurrences of `workerOptions` with the captured object literal
    string(REGEX REPLACE "workerOptions" "{ type: \"module\",name: \"em-pthread\" }" content "${content}")

    # Write the modified content back to the file
    file(WRITE "${filepath}" "${content}")
        
endfunction()

fixWasm("${filepath}")