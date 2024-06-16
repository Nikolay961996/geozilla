import React, {useState} from 'react';
import {Box, Button, Container, Snackbar} from "@mui/material";
import FileUploader from "./FileUploader";
import CoordinateInput from "./CoordinateInput";
import {LatLngString} from "../types/LatLng";
import {GenerateGeoJsonApi} from "../api/GenerateGeoJsonApi";
import {GeoJsonObject} from "geojson";

interface DataSenderProps {
    setGeoJson: React.Dispatch<React.SetStateAction<GeoJsonObject | null>>;
}

const DataSender: React.FC<DataSenderProps> = ({setGeoJson}) => {
    const [uploadedFile, setUploadedFile] = useState<File | null>(null);
    const [selectedCoords, setSelectedCoords] = useState<LatLngString | null>(null);
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = () => {
        if (!selectedCoords || !uploadedFile)
            return;

        GenerateGeoJsonApi.sendData(selectedCoords, uploadedFile)
            .then(async response => {
                if (response) {
                    setSnackbarOpen(true);
                    setSelectedCoords({lat: '', lng: ''});
                    setUploadedFile(null);
                    setError('');
                    setGeoJson(JSON.parse(await response.data.text()));
                } else {
                    throw new Error('Не удалось отправить координаты');
                }
            })
            .catch(error => {
                console.error(error);
                setError('Произошла ошибка при отправке координат');
            });
    };

    return (
        <Container>

            <Box marginTop={4}>
                <CoordinateInput selectedCoords={selectedCoords} setSelectedCoords={setSelectedCoords}/>
            </Box>

            {error && <p style={{ color: 'red' }}>{error}</p>}
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="Данные успешно отправлены"
            />

            { !uploadedFile && <Box margin={1}>Выберите файл</Box> }
            <Box marginTop={1} marginBottom={1}>
                <FileUploader setUploadedFile={setUploadedFile} />
            </Box>
            <Button variant="contained" onClick={handleSubmit}>
                Отправить данные
            </Button>
        </Container>
    );
};

export default DataSender;