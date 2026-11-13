# Open the registered orchard point clouds in Rhino 8

The current point-cloud download includes native Rhino files for all four systems. The complete sensor-data PLYs remain unchanged.

1. Download and extract [01_final_aligned_pointclouds.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointclouds.zip).
2. In Rhino 8, use **File → Open** to open `<dataset>/rhino/final_registered.3dm` as a new document.
3. If necessary, select all and run **Zoom Selected** (`_Zoom _Selected`).

The native document uses **millimetres**. A source coordinate of 1 metre becomes 1000 millimetres; the physical shape, pose, scale and distances are unchanged. Your ShrinkWrap values of 15 mm target edge length and −5 mm offset can be entered as **15** and **−5** in this document.

Each original scan is a visible point-cloud object on its own `scan_000`, `scan_001`, … layer. The object carries its source scan name and ID. Original green/brown point colours are preserved. Raw SIGNAL/intensity is stored in the native cloud's **Point Values**; other sensor channels and per-return IDs remain in the complete parent PLY. Select all scan-cloud objects together when applying ShrinkWrap to the full tree row.

## Simple PLY fallback

`<dataset>/rhino/final_registered_xyzrgb_ascii.ply` contains only standard `x y z red green blue` properties. It is text (ASCII), with no custom binary fields. **Its coordinates are in metres.** Select metres as the source units if Rhino asks when importing it into a millimetre document.

The parent `<dataset>/final_registered.ply` remains the full analysis file, in metres, retaining intensity/SIGNAL, NIR, reflectivity, scan IDs, frame IDs, pixel indices and semantic scores. No fields have been stripped from it. Transformation matrices continue to use metres and already apply to the parent/fallback PLY coordinates. Native coordinates are those same final coordinates multiplied by 1000 for their explicitly declared millimetre model units.

## What was checked

All four original files have finite coordinates within a few metres of the origin and internally correct packed binary record sizes (47 bytes per vertex). MATLAB and an independent Open3D read both produce the expected coordinates; Open3D also reproduces RGB exactly. This does **not** reproduce or identify the exact failure in the user's Rhino build. Extra PLY properties are valid under the format, but some import/display paths may handle them differently.

The native copies are written using McNeel's `rhino3dm` library, then reopened. Every point's native XYZ equals the original XYZ ×1000, every RGB triplet is unchanged, raw intensity values agree, and scan grouping/counts agree. The clouds are valid, coloured and contain no hidden points. The ASCII fallback is separately read back and every XYZ/RGB value compared exactly. Per-dataset source/output hashes, counts and bounds are in `rhino/export_quality.json`.

The Rhino 8 desktop UI is not available in this Linux environment. File-format/read-back verification is complete; successful rendering in the user's Rhino installation remains to be confirmed. If the native file still fails to display, the exact Rhino service release, Windows/Mac version and `SystemInfo` output would help distinguish an application/display issue from file parsing. McNeel has documented several different PLY import/display symptoms; they do not establish the cause here.

Sources: [Rhino 8 PLY import/export](https://docs.mcneel.com/rhino/8/help/en-us/fileio/polygon_file_format_ply_import_export.htm), [McNeel rhino3dm PointCloud API](https://mcneel.github.io/rhino3dm/python/api/PointCloud.html), [McNeel Rhino 8 PLY import discussion](https://discourse.mcneel.com/t/rh8-beta-import-of-ply-pointcloud-not-working/169378).

## Reproduce the compatibility exports

```bash
python -m pip install -r requirements-rhino.txt
python tools/export_rhino.py --pointclouds-root /path/to/extracted_pointcloud_zip
```

This is a format conversion only. It imports no registration or meshing modules, runs no alignment, removes no samples and does not rewrite the source PLY. Native files use McNeel's Rhino 8 archive format, with no invented surface normals.

`tools/publish_rhino_exports.py` downloads the checksum-verified current point-cloud ZIP, generates the native/ASCII copies, checks every pre-existing file is byte-identical, and replaces only that same asset after validation. It verifies that the mesh, matrices, QC and contribution assets retain their original asset IDs, sizes and checksums. The geometry tag remains unchanged. The dedicated GitHub workflow uses the `[rhino-exports]` commit marker to perform this compatibility-only publication; the main registration workflow skips that marker. Future full builds also generate these exports after registration.
