# Proposed Dashboard Layout

## Grid: 3 columns × 3 rows

```
+---------------------+------------------+---------------------+
|                     |                  |                     |
|  Rerun 3D           |  Skills Feed     |                     |
|  (col1, rows 1-2)   |  (col2, row1)    |  [EMPTY]            |
|                     |                  |  (col3, rows 1-3)   |
|                     +------------------+                     |
|                     |                  |                     |
|                     |  Claude          |                     |
|                     |  (col2, row2)    |                     |
+----------+----------+------------------+                     |
|          |          |                  |                     |
| LCM Stats|  dtop    |  MCP Skills      |                     |
| (col1-L) | (col1-R) |  (col2, row3)    |                     |
|  row3    |  row3    |                  |                     |
+----------+----------+------------------+---------------------+
```

## What changed from current

| Slot | Was | Now |
|------|-----|-----|
| Col1, rows 1-2 | Command Center | Rerun 3D |
| Col1, row 3 | LCM Stats | LCM Stats + dtop (split) |
| Col2, row 1 | Rerun 3D | Skills Feed |
| Col2, row 2 | Rerun 3D | Claude |
| Col2, row 3 | dtop | MCP Skills |
| Col3, row 1 | Skills Feed | **Empty** |
| Col3, row 2 | Claude | **Empty** |
| Col3, row 3 | MCP Skills | **Empty** |

## Removed

- Command Center (2D map iframe)

## Empty slot (col3)

Full-height empty panel spanning all 3 rows. Reserved for Phase 4 (People Intelligence — person cards, activity tracking).
