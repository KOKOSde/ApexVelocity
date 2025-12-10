// Package main is the entry point for the ApexVelocity REST API server.
package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/apexvelocity/server/internal/api"
	"github.com/apexvelocity/server/internal/solver"
	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"github.com/go-chi/cors"
)

func main() {
	// Command-line flags
	port := flag.Int("port", 8080, "Server port")
	configDir := flag.String("config", "", "Path to config directory")
	workers := flag.Int("workers", 4, "Number of worker goroutines for CPU-bound tasks")
	flag.Parse()

	// Initialize configuration
	if *configDir == "" {
		// Try to find config directory relative to executable
		*configDir = "../config"
	}

	log.Printf("Initializing ApexVelocity server...")
	log.Printf("Config directory: %s", *configDir)
	log.Printf("Worker pool size: %d", *workers)

	if err := solver.InitConfig(*configDir); err != nil {
		log.Printf("Warning: Failed to initialize config: %v", err)
		log.Printf("Using default configuration")
	}

	// Create API handler
	handler := api.NewHandler(*workers)

	// Setup router
	r := chi.NewRouter()

	// Middleware
	r.Use(api.LoggingMiddleware)
	r.Use(middleware.Recoverer)
	r.Use(middleware.RealIP)
	r.Use(middleware.Timeout(60 * time.Second))

	// CORS
	r.Use(cors.Handler(cors.Options{
		AllowedOrigins:   []string{"*"},
		AllowedMethods:   []string{"GET", "POST", "PUT", "DELETE", "OPTIONS"},
		AllowedHeaders:   []string{"Accept", "Authorization", "Content-Type"},
		ExposedHeaders:   []string{"Link"},
		AllowCredentials: false,
		MaxAge:           300,
	}))

	// Routes
	r.Get("/health", handler.HandleHealth)

	r.Route("/v1", func(r chi.Router) {
		r.Post("/config/reload", handler.HandleConfigReload)
		r.Post("/analyze", handler.HandleAnalyze)
	})

	// Start server
	addr := fmt.Sprintf(":%d", *port)
	srv := &http.Server{
		Addr:         addr,
		Handler:      r,
		ReadTimeout:  30 * time.Second,
		WriteTimeout: 60 * time.Second,
		IdleTimeout:  120 * time.Second,
	}

	// Graceful shutdown
	go func() {
		sigChan := make(chan os.Signal, 1)
		signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)
		<-sigChan
		log.Println("Shutting down server...")
		srv.Close()
	}()

	log.Printf("ApexVelocity server starting on %s", addr)
	log.Printf("Endpoints:")
	log.Printf("  GET  /health           - Health check")
	log.Printf("  POST /v1/config/reload - Reload configuration")
	log.Printf("  POST /v1/analyze       - Analyze path velocity profile")

	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatalf("Server error: %v", err)
	}

	log.Println("Server stopped")
}





