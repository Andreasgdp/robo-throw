# Do data extraction

## install requirements

```python
python -m pip install -r requirements.txt
```

## Get the data

- Load the `data-dump.sql` into a database in e.g. MySQL workbench
- Run the `data extraction roboThrow.sql` query on the database
- Export the data to a CSV file

## Analyze the data

- Run the python script `roboThrow.py`