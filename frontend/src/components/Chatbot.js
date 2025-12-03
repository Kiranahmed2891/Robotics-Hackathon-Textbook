// frontend/src/components/Chatbot.js

import React, { useState, useEffect } from 'react';
import axios from 'axios';
import styles from './Chatbot.module.css'; // Will create this later (T4.3)

const API_BASE_URL = 'http://127.0.0.1:8000/api/v1';

const Chatbot = () => {
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState([]); // Array to store chat history
  const [status, setStatus] = useState({ text: 'Initializing...', type: 'info' });
  const [isProcessing, setIsProcessing] = useState(false);
  const [isIngested, setIsIngested] = useState(false);

  useEffect(() => {
    // Initial health check on load
    checkHealth();
  }, []);

  // --- Health Check ---
  const checkHealth = async () => {
    try {
      const response = await axios.get(`http://127.0.0.1:8000/health`); // Note: Using 8000/health directly
      if (response.data.status === 'ok') {
        setStatus({ text: 'Backend is healthy. Ready to ingest documents.', type: 'success' });
        // After health check, try to ingest (if not already done)
        if (!isIngested) {
            ingestDocuments();
        }
      }
    } catch (error) {
      setStatus({ text: 'Error connecting to backend API (http://127.0.0.1:8000). Ensure Uvicorn is running.', type: 'error' });
    }
  };

  // --- Document Ingestion ---
  const ingestDocuments = async () => {
    setIsProcessing(true);
    setStatus({ text: 'Ingesting documents into Vector DB (this may take a minute)...', type: 'loading' });
    try {
      // POST request to trigger the ingestion pipeline
      const response = await axios.post(`${API_BASE_URL}/ingest`);
      if (response.data.status === 'success') {
        setStatus({ text: `Ingestion successful! ${response.data.chunks_ingested} chunks ready.`, type: 'success' });
        setIsIngested(true);
        // Initial bot greeting
        setMessages([{ sender: 'bot', text: 'Hello! I am your RAG Chatbot. Ask me about the technical textbook content.' }]);
      } else {
        setStatus({ text: `Ingestion failed: ${response.data.message}`, type: 'error' });
      }
    } catch (error) {
      setStatus({ text: `Ingestion failed. Ensure Qdrant container is running: ${error.message}`, type: 'error' });
    } finally {
      setIsProcessing(false);
    }
  };

  // --- Chat Submission ---
  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim() || isProcessing || !isIngested) return;

    const userMessage = input;
    setInput('');
    setMessages((prev) => [...prev, { sender: 'user', text: userMessage }]);
    setIsProcessing(true);
    
    try {
      // POST request to the RAG chat endpoint
      const response = await axios.post(`${API_BASE_URL}/chat`, { query: userMessage });
      const botAnswer = response.data.answer;
      const context = response.data.context;

      setMessages((prev) => [...prev, { 
        sender: 'bot', 
        text: botAnswer, 
        context: context // Store context for display
      }]);

    } catch (error) {
      setMessages((prev) => [...prev, { sender: 'bot', text: 'Sorry, I encountered an error while processing your request.' }]);
    } finally {
      setIsProcessing(false);
    }
  };

  // --- Rendering ---
  const renderMessage = (msg, index) => (
    <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
      <div className={styles.messageBubble}>
        {msg.text}
      </div>
      {msg.sender === 'bot' && msg.context && msg.context.length > 0 && (
        <div className={styles.contextInfo}>
          <details>
            <summary>Sources Used ({msg.context.length})</summary>
            <ul>
              {msg.context.map((ctx, ctxIndex) => (
                <li key={ctxIndex}>
                  <strong>Source:</strong> {ctx.chapter} ({ctx.source.split('/').pop()})<br />
                  <span className={styles.snippet}>Snippet: "{ctx.text_snippet}"</span>
                </li>
              ))}
            </ul>
          </details>
        </div>
      )}
    </div>
  );


  return (
    <div className={styles.chatContainer}>
      <div className={styles.statusBanner} data-type={status.type}>
        {status.text}
        {status.type === 'error' && <button onClick={checkHealth} className={styles.retryButton}>Retry</button>}
      </div>

      <div className={styles.messageList}>
        {messages.map(renderMessage)}
      </div>

      <form className={styles.inputForm} onSubmit={handleSubmit}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder={isIngested ? "Ask your question here..." : "Waiting for ingestion..."}
          disabled={isProcessing || !isIngested}
        />
        <button type="submit" disabled={isProcessing || !isIngested}>
          {isProcessing ? 'Thinking...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default Chatbot;