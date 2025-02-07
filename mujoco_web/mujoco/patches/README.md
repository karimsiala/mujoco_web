## Patches

## Summary

In order to build MuJoCo into WebAssembly we must apply some patches as described in this post:

https://github.com/stillonearth/MuJoCo-WASM/issues/1#issuecomment-1495814568

## Creating a patch with git diff

To create a patch with `git diff`, place you old code into a folder called `a` and the new code into a folder called `b`.

Then run the following command on the root folder:

```bash
git diff --no-index --src-prefix= --dst-prefix= a/ b/ > changes.patch
```
