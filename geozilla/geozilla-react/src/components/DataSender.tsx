import React, {useState} from 'react';
import {Box, Button, Container, Snackbar} from "@mui/material";
import FileUploader from "./FileUploader";
import CoordinateInput from "./CoordinateInput";
import {LatLngString} from "../types/LatLng";
import {GenerateGeoJsonApi} from "../api/GenerateGeoJsonApi";

const DataSender = () => {
    const [uploadedFile, setUploadedFile] = useState<File | null>(null);
    const [selectedCoords, setSelectedCoords] = useState<LatLngString | null>(null);
    const [snackbarOpen, setSnackbarOpen] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = () => {
        if (!selectedCoords || !uploadedFile)
            return;

        GenerateGeoJsonApi.sendData(selectedCoords, uploadedFile)
            .then(response => {
                console.log(response);

                if (response) {
                    setSnackbarOpen(true);
                    setSelectedCoords({lat: '', lng: ''});
                    setUploadedFile(null);
                    setError('');
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
                <FileUploader setUploadedFile={setUploadedFile} />
            </Box>
            <Box marginTop={4}>
                <CoordinateInput selectedCoords={selectedCoords} setSelectedCoords={setSelectedCoords}/>
            </Box>
            <Button variant="contained" onClick={handleSubmit}>
                Отправить данные
            </Button>
            {error && <p style={{ color: 'red' }}>{error}</p>}
            <Snackbar
                open={snackbarOpen}
                onClose={() => setSnackbarOpen(false)}
                message="Данные успешно отправлены"
            />

            <div>
                <div>
                    {
                        uploadedFile && <div>File: {uploadedFile?.name}, {uploadedFile?.size}</div>
                    }
                    {
                        !uploadedFile && <div>Выберите файл</div>
                    }
                </div>
                <div>
                    {
                        selectedCoords && <div>{selectedCoords.lat}; {selectedCoords.lng}</div>
                    }
                </div>
            </div>
        </Container>
    );
};

export default DataSender;