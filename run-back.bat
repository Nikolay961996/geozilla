echo 'Run Backend'
cd geozilla

echo 'Build cpp...'

echo 'Start web api...'
dotnet run --project ./geozilla-api/geozilla-api.csproj

PAUSE
